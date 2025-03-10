#include "stm32g0xx_hal.h"
#include "tmc2209_d.h"

//#define BUFFER_LEN 255

//running master mode, reading is not possible on SKR Mini e3 v3

USART_HandleTypeDef TMC2209_huart;
uint8_t TMC2209_write_reg_msg[8];
//uint8_t TMC2209_read_reg_msg[4];
//uint8_t TMC2209_read_reg_response[8];
//uint8_t TMC2209_write_ind;
//uint8_t TMC2209_read_ind;
//uint8_t TMC2209_rcv_buffer[BUFFER_LEN+1];

TMC2209_CONF_t TMC2209_motors[4];

static uint8_t calc_tmc2209_crc_byte(uint8_t* datagram, uint8_t data_len){ //directly copied from TMC2209 documentation
    uint8_t i,j;
    uint8_t crc = 0; 
    uint8_t currentByte;
    for (i=0; i<data_len; i++) { // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j=0; j<8; j++) {
            if ((crc >> 7) ^ (currentByte&0x01)){ // update CRC based result of XOR operation
                crc = (crc << 1) ^ 0x07;
            }
            else {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
    return crc;
}

/*HAL_StatusTypeDef TMC2209_ReadRegister(uint8_t addr, uint8_t reg, uint32_t *value) { //addr 0-3 correspond to motor number 1-4
	TMC2209_read_reg_msg[0] = TMC2209_SYNC_BYTE; // Sync byte
    TMC2209_read_reg_msg[1] = addr; // Slave address
    TMC2209_read_reg_msg[2] = reg & 0x7F; // Read bit clear
    TMC2209_read_reg_msg[3] = calc_tmc2209_crc_byte(TMC2209_read_reg_msg, 3);

    TMC2209_write_ind = 0;
    TMC2209_read_ind = 0;
    HAL_UART_Receive_IT(&TMC2209_huart, &TMC2209_rcv_buffer[TMC2209_write_ind], 12);  //start reception
    //HAL_Delay(1);
    if (HAL_UART_Transmit(&TMC2209_huart, TMC2209_read_reg_msg, 4, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    //HAL_Delay(1);
    while (TMC2209_write_ind){} //wait for echo(4 bytes) + response(8 bytes), signalled by write_ind > 0
    TMC2209_read_ind = TMC2209_read_ind + 4; //skip echo
    for (int i = 0; i<8; i++){
    	TMC2209_read_reg_response[i] = TMC2209_rcv_buffer[TMC2209_read_ind];
    	TMC2209_read_ind++;
    }
    if (TMC2209_read_reg_response[7] != calc_tmc2209_crc_byte(TMC2209_read_reg_response, 7)) {
        return HAL_ERROR; // CRC mismatch
    }
    *value = (TMC2209_read_reg_response[3] << 24) | (TMC2209_read_reg_response[4] << 16) | (TMC2209_read_reg_response[5] << 8) | TMC2209_read_reg_response[6];
    return HAL_OK;
}*/

HAL_StatusTypeDef TMC2209_WriteRegister(uint8_t addr, uint8_t reg, uint32_t value) { //addr 0-3 correspond to motor number 1-4
    HAL_StatusTypeDef result;
    TMC2209_write_reg_msg[0] = TMC2209_SYNC_BYTE; // Sync byte
    TMC2209_write_reg_msg[1] = addr; // Slave address
    TMC2209_write_reg_msg[2] = reg | 0x80; // Write bit set
    TMC2209_write_reg_msg[3] = (value >> 24) & 0xFF;
    TMC2209_write_reg_msg[4] = (value >> 16) & 0xFF;
    TMC2209_write_reg_msg[5] = (value >> 8) & 0xFF;
    TMC2209_write_reg_msg[6] = value & 0xFF;
    TMC2209_write_reg_msg[7] = calc_tmc2209_crc_byte(TMC2209_write_reg_msg, 7);
/*
    TMC2209_write_ind = 0;
    TMC2209_read_ind = 0;
    HAL_UART_Receive_IT(&TMC2209_huart, &TMC2209_rcv_buffer[TMC2209_write_ind], 8);  //start reception
*/
    result = HAL_USART_Transmit(&TMC2209_huart, TMC2209_write_reg_msg, 8, HAL_MAX_DELAY);
/*
    while (TMC2209_write_ind){} //wait for echo(8 bytes), signalled by write_ind > 0
    TMC2209_read_ind = TMC2209_read_ind + 8; //skip echo
*/
    return result;
}

HAL_StatusTypeDef TMC2209_Init(USART_HandleTypeDef huart) {
    TMC2209_huart = huart;
    //TMC2209_write_ind = 0;
    //TMC2209_read_ind = 0;
    for (int i=0; i<4; i++){
    	TMC2209_motors[i].addr_motor = i; //addr 0 for motor1, addr 1 for motor2 and so on.

    	//TMC2209_ReadRegister(TMC2209_motors[i].addr_motor, reg_GSTAT, &TMC2209_motors[i].GSTAT.val);

        TMC2209_motors[i].GSTAT.fields.reset = 0;
        TMC2209_motors[i].GSTAT.fields.drv_err = 0;
        TMC2209_motors[i].GSTAT.fields.uv_cp = 0;
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_GSTAT, TMC2209_motors[i].GSTAT.val);

    	TMC2209_motors[i].GCONF.val = 0;
    	TMC2209_motors[i].GCONF.fields.I_scale_analog = 1;
    	TMC2209_motors[i].GCONF.fields.internal_Rsense = 1;
        TMC2209_motors[i].GCONF.fields.en_SpreadCycle = 0; //0: stealthchop, 1: spreadcycle
        TMC2209_motors[i].GCONF.fields.mstep_reg_select = 1; //mres by uart
        TMC2209_motors[i].GCONF.fields.shaft = 0; //0: stealthchop, 1: spreadcycle
        TMC2209_motors[i].GCONF.fields.index_otpw = 1;
        TMC2209_motors[i].GCONF.fields.index_step = 0;
        TMC2209_motors[i].GCONF.fields.pdn_disable = 1;
        TMC2209_motors[i].GCONF.fields.mstep_reg_select = 1;
        TMC2209_motors[i].GCONF.fields.multistep_filt = 1;
        TMC2209_motors[i].GCONF.fields.test_mode = 0;
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_GCONF, TMC2209_motors[i].GCONF.val);

        TMC2209_motors[i].PWMCONF.val = 0xC10D0024;
        TMC2209_motors[i].PWMCONF.fields.freewheel = 2; //short to LS
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_PWMCONF, TMC2209_motors[i].PWMCONF.val);

        TMC2209_motors[i].CHOPCONF.val = 0x10000053;
        TMC2209_motors[i].CHOPCONF.fields.intpol = 1;
        TMC2209_motors[i].CHOPCONF.fields.mres = TMC2209_MSTEP1;
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_CHOPCONF, TMC2209_motors[i].CHOPCONF.val);

        TMC2209_motors[i].IHOLD_IRUN.val = 0;
        TMC2209_motors[i].IHOLD_IRUN.fields.ihold = 0;
        TMC2209_motors[i].IHOLD_IRUN.fields.iholddelay = 1;
        TMC2209_motors[i].IHOLD_IRUN.fields.irun = 31;
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_IHOLD_IRUN, TMC2209_motors[i].IHOLD_IRUN.val);

        TMC2209_motors[i].TCOOLTHRS.fields.tcoolthrs = 0; //disable
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_TCOOLTHRS, TMC2209_motors[i].TCOOLTHRS.val);

        TMC2209_motors[i].TPWMTHRS.fields.tpwmthrs = 0; //disable
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_TPWMTHRS, TMC2209_motors[i].TPWMTHRS.val);

        TMC2209_motors[i].TPOWERDOWN.fields.tpowerdown = 255; //power down after ~1sec = (5.6secs/255)*40
        TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_TPOWERDOWN, TMC2209_motors[i].TPOWERDOWN.val);

    	TMC2209_motors[i].VACTUAL.fields.vactual = 0;
    	TMC2209_WriteRegister(TMC2209_motors[i].addr_motor, reg_VACTUAL, TMC2209_motors[i].VACTUAL.val);
    }
    return HAL_OK;
}

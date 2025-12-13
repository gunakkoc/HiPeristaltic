// Copyright 2025 Gun Deniz Akkoc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// https://github.com/gunakkoc/HiPeristaltic

#include <HardwareSerial.h>

#define BUFFER_LEN 24                                 //suitable for Ethernet shield
#define SERIAL_INTERBYTE_TIMEOUT_US 500000L
#define MOTOR_MIN_PULSE_WIDTH_US 3L //1us for A4988, 2us for DRV8825, ~100ns for TMC2208 and TMC2209

#ifndef TMC2209_driver
#define TMC2209_driver
#endif

HardwareSerial Serial5(USART5);
HardwareSerial Serial3(USART3);

const uint32_t SUB_US_DIV = 1;
const uint32_t SERIAL_INTERBYTE_TIMEOUT = SERIAL_INTERBYTE_TIMEOUT_US * SUB_US_DIV;
const uint32_t MOTOR_MIN_PULSE_WIDTH = MOTOR_MIN_PULSE_WIDTH_US * SUB_US_DIV; //Arduino 16MHz clock is so slow that we don't need to check this
const uint8_t CMD_COUNT = 68;

const uint8_t MSG_LEN = 6;
uint8_t rcv_buffer[BUFFER_LEN];
uint8_t snd_buffer[BUFFER_LEN];
uint8_t rcv_byte_cnt = 0;
uint8_t snd_byte_cnt = MSG_LEN;
uint32_t rcv_last_tick = 0;
uint8_t checksum = 0;

const uint32_t min2us = 60000000L;

// ------- START OF MOTOR PINS AND VARIABLES
const int m0_enabled_pin = PB14;
const int m0_dir_pin = PB12;
const int m0_step_pin = PB13;

bool m0_running = false;
uint32_t m0_tick_last = 0L;
uint32_t m0_steps = 0L;
uint32_t m0_target_steps = 0L;
uint8_t m0_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m0_last_pulse = LOW;
uint8_t m0_usteps_exp = 0;

bool m0_enabled_pin_state = true;
bool m0_dir_pin_state = false;
uint32_t m0_step_interval = 4000L;

// -----------------m1---------------------
const int m1_enabled_pin = PB11;
const int m1_dir_pin = PB2;
const int m1_step_pin = PB10;

bool m1_running = false;
uint32_t m1_tick_last = 0L;
uint32_t m1_steps = 0L;
uint32_t m1_target_steps = 0L;
uint8_t m1_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m1_last_pulse = LOW;
uint8_t m1_usteps_exp = 0;

bool m1_enabled_pin_state = true;
bool m1_dir_pin_state = false;
uint32_t m1_step_interval = 4000L;

//-----------------m2---------------------
const int m2_enabled_pin = PB1;
const int m2_dir_pin = PC5;
const int m2_step_pin = PB0;

bool m2_running = false;
uint32_t m2_tick_last = 0L;
uint32_t m2_steps = 0L;
uint32_t m2_target_steps = 0L;
uint8_t m2_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m2_last_pulse = LOW;
uint8_t m2_usteps_exp = 0;

bool m2_enabled_pin_state = true;
bool m2_dir_pin_state = false;
uint32_t m2_step_interval = 4000L;

//-----------------m3--------------------- //FOR E1: EN D30, DIR D34, STP D36
const int m3_enabled_pin = PD1; //E0
const int m3_dir_pin = PB4;
const int m3_step_pin = PB3;

bool m3_running = false;
uint32_t m3_tick_last = 0L;
uint32_t m3_steps = 0L;
uint32_t m3_target_steps = 0L;
uint8_t m3_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m3_last_pulse = LOW;
uint8_t m3_usteps_exp = 0;

bool m3_enabled_pin_state = true;
bool m3_dir_pin_state = false;
uint32_t m3_step_interval = 4000L;
// ------- END OF MOTOR PINS AND VARIABLES

// ------- START OF TMC2209 VARIABLES
uint8_t TMC2209_ustep_exp_int_temp = 0;
uint32_t TMC2209_ustep_exp_bits_temp = 0;
// ------- END OF TMC2209 VARIABLES

// -------- START OF TMC2209 Driver ------------
typedef union CONF_GCONF { //n = 10, RW
    struct {
        uint32_t I_scale_analog : 1; //def 1
        uint32_t internal_Rsense : 1;
        uint32_t en_SpreadCycle : 1; //0 for StealthChop PWM mode voltage control, 1 for SpreadCycle current control. def 0
        uint32_t shaft : 1;
        uint32_t index_otpw : 1;
        uint32_t index_step : 1;
        uint32_t pdn_disable : 1;
        uint32_t mstep_reg_select : 1; //set to 1 for UART control of mres, otherwise pin controlled.
        uint32_t multistep_filt : 1; //def 1
        uint32_t test_mode : 1; //set 0
        uint32_t empty : 22;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_GCONF_t;

typedef union CONF_GSTAT { //n = 3, RW(C)
    struct {
        uint32_t reset : 1; //resetted
        uint32_t drv_err : 1; //critical error, write 1 to clear
        uint32_t uv_cp : 1; //overcurrent, no rewriting to clear
        // uint32_t openload_a : 1;
        // uint32_t openload_b : 1;
        // uint32_t short_a : 1;
        // uint32_t short_b : 1;
        // uint32_t s2ga : 1;
        uint32_t empty : 29;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_GSTAT_t;

typedef union CONF_CHOPCONF { //n = 32, RW
    struct {
        uint32_t toff : 4; //toff0, toff1, toff2, toff3
        uint32_t hstrt : 3; //hs0, hs1, hs2
        uint32_t hend : 4; //hend0, hend1, hend2, hend3
        uint32_t reserved_1 : 4; //0
        uint32_t tbl : 2; //tbl0, tbl1
        uint32_t vsense : 1; 
        uint32_t reserved_2 : 6; //0
        uint32_t mres : 4; //mres0, mres1, mres2, mres3
        uint32_t intpol : 1; //def 1
        uint32_t dedge : 1;
        uint32_t diss2g : 1;
        uint32_t diss2vs : 1;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_CHOPCONF_t;

typedef union CONF_VACTUAL { // n = 24, W
//VACTUAL = v[Hz] / ( f CLK [Hz] / 2^24 ) = v[Hz] / 0.715Hz
    struct {
        int32_t vactual : 24; //signed +/- 2^23 also with int32 the sign extension is done and uint32 will not do sign extension
        uint32_t empty : 8;
    } fields;
    uint32_t val; //signed +/- 2^23
    uint8_t byte_arr[4];
} CONF_VACTUAL_t;

typedef union CONF_MSCNT { // n = 10, R
    struct {
        uint32_t mscnt : 10;
        uint32_t empty : 22;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_MSCNT_t;

typedef union CONF_IHOLD_IRUN { //n = 32, W
    struct {
        uint32_t ihold : 5; //def 16
        uint32_t reserved_1 : 3;
        uint32_t irun : 5; //def 31
        uint32_t reserved_2 : 3;
        uint32_t iholddelay : 4; // def 1 but can be 2 as well, not 0
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_IHOLD_IRUN_t;

typedef union CONF_TPOWERDOWN { // n = 8, W
    struct {
        uint32_t tpowerdown : 8; //def 20
        uint32_t empty : 24;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_TPOWERDOWN_t;

typedef union CONF_PWMCONF { // n = 32, RW
    struct {
        uint32_t pwm_ofs : 8; //maybe read initial val from PWM_OFS_AUTO
        uint32_t pwm_grad : 8; //maybe read initial val from PWM_GRAD_AUTO
        uint32_t pwm_freq : 2;
        uint32_t pwm_autoscale : 1;
        uint32_t pwm_autograd : 1;
        uint32_t freewheel : 2; //0: normal, 1: freewheel, 2: coil short to LS drivers, 3: coil short to HS drivers
        uint32_t reserved : 2;
        uint32_t pwm_reg : 4;
        uint32_t pwm_lim : 4;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_PWMCONF_t;

typedef union CONF_PWMAUTO { // n = 16, R
    struct {
        uint32_t pwm_ofs_auto : 8; //might be useful to store
        uint32_t pwm_grad_auto : 8; //might be useful to store
        uint32_t empty : 16;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_PWMAUTO_t;

typedef union CONF_TPWMTHRS { // n = 20, W
    struct {
        uint32_t tpwmthrs : 20; //if TPWMTHRS < TSTEP then stealthChop is disabled, spreadCycle is enabled. set 999999 on official firmware for klipper, 0 def disables this
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_TPWMTHRS_t;

typedef union CONF_TSTEP { // n = 20, R
    struct {
        uint32_t tstep : 20; //time between steps
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_TSTEP_t;

typedef union CONF_TCOOLTHRS { // n = 20, W
    struct {
        uint32_t tcoolthrs : 20;
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_TCOOLTHRS_t;

typedef union CONF_DRV_STATUS { // n = 32, R
    struct {
        uint32_t otpw : 1;
        uint32_t ot : 1;
        uint32_t s2ga : 1;
        uint32_t s2gb : 1;
        uint32_t s2vsa : 1;
        uint32_t s2vsb : 1;
        uint32_t ola : 1;
        uint32_t t120 : 1;
        uint32_t t143 : 1;
        uint32_t t150 : 1;
        uint32_t t157 : 1;
        uint32_t reserved_1 : 4;
        uint32_t cs_actual : 5;
        uint32_t reserved_2 : 9;
        uint32_t stealth : 1;
        uint32_t stst : 1;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_DRV_STATUS_t;

typedef struct TMC2209_CONF {
  CONF_GCONF_t GCONF; //rw
  CONF_GSTAT_t GSTAT; //rw(c)
  CONF_CHOPCONF_t CHOPCONF; //rw
  CONF_VACTUAL_t VACTUAL; //w only
  CONF_MSCNT_t MSCNT; //r only
  CONF_IHOLD_IRUN_t IHOLD_IRUN; //w only
  CONF_TPOWERDOWN_t TPOWERDOWN; //w only
  CONF_PWMCONF_t PWMCONF; //rw
  CONF_PWMAUTO_t PWMAUTO; //r only
  CONF_DRV_STATUS_t DRV_STATUS;
  CONF_TPWMTHRS_t TPWMTHRS; //w only
  CONF_TSTEP_t TSTEP; //r only
  CONF_TCOOLTHRS_t TCOOLTHRS; //w only
  uint8_t addr_motor;
} TMC2209_CONF_t;


//in CHOPCONF
const static uint8_t TMC2209_MSTEP256 = 0b0000;
const static uint8_t TMC2209_MSTEP128 = 0b0001;
const static uint8_t TMC2209_MSTEP64 = 0b0010;
const static uint8_t TMC2209_MSTEP32 = 0b0011;
const static uint8_t TMC2209_MSTEP16 = 0b0100;
const static uint8_t TMC2209_MSTEP8 = 0b0101;
const static uint8_t TMC2209_MSTEP4 = 0b0110;
const static uint8_t TMC2209_MSTEP2 = 0b0111;
const static uint8_t TMC2209_MSTEP1 = 0b1000;

static const uint8_t reg_GCONF = 0x00;
static const uint8_t reg_GSTAT = 0x01;
static const uint8_t reg_CHOPCONF = 0x6C;
static const uint8_t reg_VACTUAL = 0x22;
static const uint8_t reg_MSCNT = 0x6A;
static const uint8_t reg_IHOLD_IRUN = 0x10;
static const uint8_t reg_TPOWERDOWN = 0x11;
static const uint8_t reg_PWMCONF = 0x70;
static const uint8_t reg_PWMAUTO = 0x72;
static const uint8_t reg_TPWMTHRS = 0x13;
static const uint8_t reg_TSTEP = 0x12;
static const uint8_t reg_TCOOLTHRS = 0x14;
static const uint8_t reg_DRV_STATUS = 0x6F;

static const uint8_t TMC2209_SYNC_BYTE = 0x05;

uint8_t TMC2209_write_reg_msg[8];

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

void TMC2209_WriteRegister(uint8_t addr, uint8_t reg, uint32_t value) { //addr 0-3 correspond to motor number 1-4
    TMC2209_write_reg_msg[0] = TMC2209_SYNC_BYTE; // Sync byte
    TMC2209_write_reg_msg[1] = addr; // Slave address
    TMC2209_write_reg_msg[2] = reg | 0x80; // Write bit set
    TMC2209_write_reg_msg[3] = (value >> 24) & 0xFF;
    TMC2209_write_reg_msg[4] = (value >> 16) & 0xFF;
    TMC2209_write_reg_msg[5] = (value >> 8) & 0xFF;
    TMC2209_write_reg_msg[6] = value & 0xFF;
    TMC2209_write_reg_msg[7] = calc_tmc2209_crc_byte(TMC2209_write_reg_msg, 7);
    Serial3.write(TMC2209_write_reg_msg, 8);
    Serial3.flush();
}

void TMC2209_Init() {
    for (int i=0; i<4; i++){
        TMC2209_motors[i].addr_motor = i; //addr 0 for motor1, addr 1 for motor2 and so on.

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
}
// -------- END OF TMC2209 Driver ------------

uint32_t tick_now = 0L;

bool led_active = false;

void calc_checksum() {
  //simple 8 bit checksum with XOR
  //appends it as last byte of snd_buffer
  checksum = snd_buffer[0] ^ snd_buffer[1] ^ snd_buffer[2] ^ snd_buffer[3] ^ snd_buffer[4];
  snd_buffer[MSG_LEN - 1] = checksum;
}

bool check_checksum() {
  //simple 8 bit checksum with XOR
  //compares to last byte of rcv_buffer
  checksum = rcv_buffer[0] ^ rcv_buffer[1] ^ rcv_buffer[2] ^ rcv_buffer[3] ^ rcv_buffer[4]; //5 bytes
  return (checksum == rcv_buffer[MSG_LEN - 1]);
}

void send_buffer(){
  calc_checksum();
  // Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  // Udp.write(snd_buffer,MSG_LEN);
  // Udp.endPacket();
  // Serial5.write(snd_buffer,MSG_LEN);
  snd_byte_cnt = 0; //send by process_commands
}

void err_checksum(){
  led_active = true;
  snd_buffer[0] = 255;
  send_buffer();
}

void err_cmd(){
  snd_buffer[0] = 254;
  send_buffer();
}

void send_ack(){
  snd_buffer[0] = 253;
  send_buffer();
}

void signal_start(){
  snd_buffer[0] = 252;
  snd_buffer[1] = 252;
  snd_buffer[2] = 252;
  snd_buffer[3] = 252;
  snd_buffer[4] = 252;
  send_buffer();
}

void signal_m0_end(){
  snd_buffer[0] = 200;
  send_buffer();
}

void signal_m1_end(){
  snd_buffer[0] = 201;
  send_buffer();
}

void signal_m2_end(){
  snd_buffer[0] = 202;
  send_buffer();
}

void signal_m3_end(){
  snd_buffer[0] = 203;
  send_buffer();
}

void get_m0_running(){
  snd_buffer[1] = m0_running;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_running(){
  if ((bool) m0_running == (bool) rcv_buffer[1]) {
    send_ack();
    return;
  }
  m0_running = rcv_buffer[1];
  if (m0_running) {
    m0_last_pulse = LOW;
    digitalWrite(m0_step_pin, LOW);
    m0_tick_last = tick_now - m0_step_interval;
  }
  send_ack();
}

void get_m0_steps(){
  * (uint32_t *) &snd_buffer[1] = m0_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_steps(){
  m0_steps = * (uint32_t *) &rcv_buffer[1];
  m0_target_steps = m0_steps;
  send_ack();
}

void get_m0_target_steps(){
  * (uint32_t *) &snd_buffer[1] = m0_target_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_target_steps(){
  m0_target_steps = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m0_step_interval(){
  * (uint32_t *) &snd_buffer[1] = m0_step_interval;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_step_interval(){
  m0_step_interval = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m0_finite_mode(){
  snd_buffer[1] = m0_finite_mode;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_finite_mode(){
  m0_finite_mode = rcv_buffer[1];
  send_ack();
}

void get_m0_dir(){
  snd_buffer[1] = m0_dir_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_dir(){
  m0_dir_pin_state = rcv_buffer[1];
  digitalWrite(m0_dir_pin,m0_dir_pin_state);
  send_ack();
}

void get_m0_enabled(){
  snd_buffer[1] = !m0_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_enabled(){
  m0_enabled_pin_state = !rcv_buffer[1];
  digitalWrite(m0_enabled_pin, m0_enabled_pin_state);
  send_ack();
}

void get_m1_running(){
  snd_buffer[1] = m1_running;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_running(){
  if ((bool) m1_running == (bool) rcv_buffer[1]) {
    send_ack();
    return;
  }
  m1_running = rcv_buffer[1];
  if (m1_running) {
    m1_last_pulse = LOW;
    digitalWrite(m1_step_pin, LOW);
    m1_tick_last = tick_now - m1_step_interval;
  }
  send_ack();
}

void get_m1_steps(){
  * (uint32_t *) &snd_buffer[1] = m1_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_steps(){
  m1_steps = * (uint32_t *) &rcv_buffer[1];
  m1_target_steps = m1_steps;
  send_ack();
}

void get_m1_target_steps(){
  * (uint32_t *) &snd_buffer[1] = m1_target_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_target_steps(){
  m1_target_steps = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m1_step_interval(){
  * (uint32_t *) &snd_buffer[1] = m1_step_interval;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_step_interval(){
  m1_step_interval = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m1_finite_mode(){
  snd_buffer[1] = m1_finite_mode;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_finite_mode(){
  m1_finite_mode = rcv_buffer[1];
  send_ack();
}

void get_m1_dir(){
  snd_buffer[1] = m1_dir_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_dir(){
  m1_dir_pin_state = rcv_buffer[1];
  digitalWrite(m1_dir_pin,m1_dir_pin_state);
  send_ack();
}

void get_m1_enabled(){
  snd_buffer[1] = !m1_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_enabled(){
  m1_enabled_pin_state = !rcv_buffer[1];
  digitalWrite(m1_enabled_pin, m1_enabled_pin_state);
  send_ack();
}

void get_m2_running(){
  snd_buffer[1] = m2_running;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_running(){
  if ((bool) m2_running == (bool) rcv_buffer[1]) {
    send_ack();
    return;
  }
  m2_running = rcv_buffer[1];
  if (m2_running) {
    m2_last_pulse = LOW;
    digitalWrite(m2_step_pin, LOW);
    m2_tick_last = tick_now - m2_step_interval;
  }
  send_ack();
}

void get_m2_steps(){
  * (uint32_t *) &snd_buffer[1] = m2_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_steps(){
  m2_steps = * (uint32_t *) &rcv_buffer[1];
  m2_target_steps = m2_steps;
  send_ack();
}

void get_m2_target_steps(){
  * (uint32_t *) &snd_buffer[1] = m2_target_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_target_steps(){
  m2_target_steps = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m2_step_interval(){
  * (uint32_t *) &snd_buffer[1] = m2_step_interval;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_step_interval(){
  m2_step_interval = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m2_finite_mode(){
  snd_buffer[1] = m2_finite_mode;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_finite_mode(){
  m2_finite_mode = rcv_buffer[1];
  send_ack();
}

void get_m2_dir(){
  snd_buffer[1] = m2_dir_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_dir(){
  m2_dir_pin_state = rcv_buffer[1];
  digitalWrite(m2_dir_pin,m2_dir_pin_state);
  send_ack();
}

void get_m2_enabled(){
  snd_buffer[1] = !m2_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_enabled(){
  m2_enabled_pin_state = !rcv_buffer[1];
  digitalWrite(m2_enabled_pin, m2_enabled_pin_state);
  send_ack();
}

void get_m3_running(){
  snd_buffer[1] = m3_running;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_running(){
  if ((bool) m3_running == (bool) rcv_buffer[1]) {
    send_ack();
    return;
  }
  m3_running = rcv_buffer[1];
  if (m3_running) {
    m3_last_pulse = LOW;
    digitalWrite(m3_step_pin, LOW);
    m3_tick_last = tick_now - m3_step_interval;
  }
  send_ack();
}

void get_m3_steps(){
  * (uint32_t *) &snd_buffer[1] = m3_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_steps(){
  m3_steps = * (uint32_t *) &rcv_buffer[1];
  m3_target_steps = m3_steps;
  send_ack();
}

void get_m3_target_steps(){
  * (uint32_t *) &snd_buffer[1] = m3_target_steps;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_target_steps(){
  m3_target_steps = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m3_step_interval(){
  * (uint32_t *) &snd_buffer[1] = m3_step_interval;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_step_interval(){
  m3_step_interval = * (uint32_t *) &rcv_buffer[1];
  send_ack();
}

void get_m3_finite_mode(){
  snd_buffer[1] = m3_finite_mode;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_finite_mode(){
  m3_finite_mode = rcv_buffer[1];
  send_ack();
}

void get_m3_dir(){
  snd_buffer[1] = m3_dir_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_dir(){
  m3_dir_pin_state = rcv_buffer[1];
  digitalWrite(m3_dir_pin,m3_dir_pin_state);
  send_ack();
}

void get_m3_enabled(){
  snd_buffer[1] = !m3_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_enabled(){
  m3_enabled_pin_state = !rcv_buffer[1];
  digitalWrite(m3_enabled_pin, m3_enabled_pin_state);
  send_ack();
}

//variable microstepping is supported through TMC2209 UART for SKR Mini e3 v3
void get_m0_var_ustep_support(){
    snd_buffer[1] = true;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void get_m1_var_ustep_support(){
    snd_buffer[1] = true;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void get_m2_var_ustep_support(){
    snd_buffer[1] = true;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void get_m3_var_ustep_support(){
    snd_buffer[1] = true;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

// ###################################### Start TMC2209 Commands ######################################

uint8_t TMC2209_usteps_exp_bits_to_int(uint32_t ustep){
  if (ustep == TMC2209_MSTEP1) {return 0;}
  if (ustep == TMC2209_MSTEP2) {return 1;}
  if (ustep == TMC2209_MSTEP4) {return 2;}
  if (ustep == TMC2209_MSTEP8) {return 3;}
  if (ustep == TMC2209_MSTEP16) {return 4;}
  if (ustep == TMC2209_MSTEP32) {return 5;}
  if (ustep == TMC2209_MSTEP64) {return 6;}
  if (ustep == TMC2209_MSTEP128) {return 7;}
  if (ustep == TMC2209_MSTEP256) {return 8;}
  return 4;
}

uint8_t TMC2209_usteps_exp_int_to_bits[] = {TMC2209_MSTEP1,TMC2209_MSTEP2,TMC2209_MSTEP4,TMC2209_MSTEP8,TMC2209_MSTEP16,TMC2209_MSTEP32,TMC2209_MSTEP64,TMC2209_MSTEP128,TMC2209_MSTEP256};

void get_m0_usteps_exp(){
  TMC2209_ustep_exp_int_temp = TMC2209_usteps_exp_bits_to_int(TMC2209_motors[0].CHOPCONF.fields.mres);
  snd_buffer[1] = TMC2209_ustep_exp_int_temp;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_usteps_exp(){
  TMC2209_motors[0].CHOPCONF.fields.mres = TMC2209_usteps_exp_int_to_bits[rcv_buffer[1]];
  TMC2209_WriteRegister(TMC2209_motors[0].addr_motor, reg_CHOPCONF, TMC2209_motors[0].CHOPCONF.val);
  send_ack();
}

void get_m1_usteps_exp(){
  TMC2209_ustep_exp_int_temp = TMC2209_usteps_exp_bits_to_int(TMC2209_motors[1].CHOPCONF.fields.mres);
  snd_buffer[1] = TMC2209_ustep_exp_int_temp;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_usteps_exp(){
  TMC2209_motors[1].CHOPCONF.fields.mres = TMC2209_usteps_exp_int_to_bits[rcv_buffer[1]];
  TMC2209_WriteRegister(TMC2209_motors[1].addr_motor, reg_CHOPCONF, TMC2209_motors[1].CHOPCONF.val);
  send_ack();
}

void get_m2_usteps_exp(){
  TMC2209_ustep_exp_int_temp = TMC2209_usteps_exp_bits_to_int(TMC2209_motors[2].CHOPCONF.fields.mres);
  snd_buffer[1] = TMC2209_ustep_exp_int_temp;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_usteps_exp(){
  TMC2209_motors[2].CHOPCONF.fields.mres = TMC2209_usteps_exp_int_to_bits[rcv_buffer[1]];
  TMC2209_WriteRegister(TMC2209_motors[2].addr_motor, reg_CHOPCONF, TMC2209_motors[2].CHOPCONF.val);
  send_ack();
}

void get_m3_usteps_exp(){
  TMC2209_ustep_exp_int_temp = TMC2209_usteps_exp_bits_to_int(TMC2209_motors[3].CHOPCONF.fields.mres);
  snd_buffer[1] = TMC2209_ustep_exp_int_temp;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_usteps_exp(){
  TMC2209_motors[3].CHOPCONF.fields.mres = TMC2209_usteps_exp_int_to_bits[rcv_buffer[1]];
  TMC2209_WriteRegister(TMC2209_motors[3].addr_motor, reg_CHOPCONF, TMC2209_motors[3].CHOPCONF.val);
  send_ack();
}
// ###################################### End TMC2209 Commands #######################################


void get_sub_us_divider(){
  * (uint32_t *) &snd_buffer[1] = SUB_US_DIV;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void (*cmd_fnc_lst[])() = {
  &get_m0_running,
  &set_m0_running,
  &get_m0_steps,
  &set_m0_steps,
  &get_m0_target_steps,
  &set_m0_target_steps,
  &get_m0_step_interval,
  &set_m0_step_interval,
  &get_m0_finite_mode,
  &set_m0_finite_mode,
  &get_m0_dir,
  &set_m0_dir,
  &get_m0_enabled,
  &set_m0_enabled,
  &get_m1_running,
  &set_m1_running,
  &get_m1_steps,
  &set_m1_steps,
  &get_m1_target_steps,
  &set_m1_target_steps,
  &get_m1_step_interval,
  &set_m1_step_interval,
  &get_m1_finite_mode,
  &set_m1_finite_mode,
  &get_m1_dir,
  &set_m1_dir,
  &get_m1_enabled,
  &set_m1_enabled,
  &get_m2_running,
  &set_m2_running,
  &get_m2_steps,
  &set_m2_steps,
  &get_m2_target_steps,
  &set_m2_target_steps,
  &get_m2_step_interval,
  &set_m2_step_interval,
  &get_m2_finite_mode,
  &set_m2_finite_mode,
  &get_m2_dir,
  &set_m2_dir,
  &get_m2_enabled,
  &set_m2_enabled,
  &get_m3_running,
  &set_m3_running,
  &get_m3_steps,
  &set_m3_steps,
  &get_m3_target_steps,
  &set_m3_target_steps,
  &get_m3_step_interval,
  &set_m3_step_interval,
  &get_m3_finite_mode,
  &set_m3_finite_mode,
  &get_m3_dir,
  &set_m3_dir,
  &get_m3_enabled,
  &set_m3_enabled,
  &get_m0_var_ustep_support,
  &get_m1_var_ustep_support,
  &get_m2_var_ustep_support,
  &get_m3_var_ustep_support,
  &get_m0_usteps_exp,
  &set_m0_usteps_exp,
  &get_m1_usteps_exp,
  &set_m1_usteps_exp,
  &get_m2_usteps_exp,
  &set_m2_usteps_exp,
  &get_m3_usteps_exp,
  &set_m3_usteps_exp,
  &get_sub_us_divider,
};

bool process_commands() {
  // if (Udp.parsePacket()) {
  //   Udp.read(rcv_buffer, BUFFER_LEN);
  // } else {
  //   return;
  // }
  if (snd_byte_cnt < MSG_LEN){ //data needs sending
    Serial5.write(snd_buffer[snd_byte_cnt]);
    snd_byte_cnt++;
    return true;
    //if (bit_is_set(UCSR0A, UDRE0)) { //only if sending a byte is possible, only works on AVR
      // UDR0 = snd_buffer[snd_byte_cnt]; //directly send a single byte, bypassing the TX buffer, only works on AVR
      // snd_byte_cnt++;
      // return true;
    // }
    // return false; //if a byte can't be send, wait for the next cycle
  } else if (rcv_byte_cnt == MSG_LEN){ //entire package is received, process
    rcv_byte_cnt = 0;
    if (check_checksum()){
    if (rcv_buffer[0] > CMD_COUNT) {
    err_cmd();
    return true;
    }
      cmd_fnc_lst[rcv_buffer[0]](); //find the corresponding func by first byte as uint8
    } else {
      err_checksum(); //request data again
    }
    return true; //continue reading (if any) on next cycle
  } else if (Serial5.available()) { //read byte by byte to buffer
      rcv_last_tick = tick_now;
      rcv_buffer[rcv_byte_cnt] = Serial5.read();
      rcv_byte_cnt++;
      return true; //process the command (if fully received) on next cycle
  } else if (rcv_byte_cnt > 0){ //handle interbyte timeout, if reached reset the buffer
    if ((tick_now - rcv_last_tick) > SERIAL_INTERBYTE_TIMEOUT) {
      rcv_byte_cnt = 0;
      return true;
    }
  }
  return false; //nothing happened
}

uint32_t rpm1motorus(int ppr, int microstepping, float rpm) {
  //typical ppr = 200step/rev
  //microstepping = 1, 2, 4, ..., 16, 32, 64
  //this is microsecs time in between motor pulses of rising edge
  return (min2us / (rpm * ppr * microstepping));
}

void m0step() {
  if (!m0_running){
    return;
  }
  if (m0_steps) {
    if (m0_last_pulse) {
      m0_last_pulse = false;
      digitalWrite(m0_step_pin, false);
      return;
    } else if ((tick_now - m0_tick_last) >= m0_step_interval) {
      digitalWrite(m0_step_pin, true);
      m0_last_pulse = true;
      m0_steps-=m0_finite_mode; //0 for continuous mode, 1 for finite steps
      m0_tick_last = tick_now;
      return;
    }
  } else if (snd_byte_cnt == MSG_LEN) { 
    m0_running = false;
    signal_m0_end();
  }
}

void m1step() {
  if (!m1_running){
    return;
  }
  if (m1_steps) {
    if (m1_last_pulse) {
      m1_last_pulse = false;
      digitalWrite(m1_step_pin, false);
      return;
    } else if ((tick_now - m1_tick_last) >= m1_step_interval) {
      digitalWrite(m1_step_pin, true);
      m1_last_pulse = true;
      m1_steps-=m1_finite_mode; //0 for continuous mode, 1 for finite steps
      m1_tick_last = tick_now;
      return;
    }
  } else if (snd_byte_cnt == MSG_LEN) { 
    m1_running = false;
    signal_m1_end();
  }
}

void m2step() {
  if (!m2_running){
    return;
  }
  if (m2_steps) {
    if (m2_last_pulse) {
      m2_last_pulse = false;
      digitalWrite(m2_step_pin, false);
      return;
    } else if ((tick_now - m2_tick_last) >= m2_step_interval) {
      digitalWrite(m2_step_pin, true);
      m2_last_pulse = true;
      m2_steps-=m2_finite_mode; //0 for continuous mode, 1 for finite steps
      m2_tick_last = tick_now;
      return;
    }
  } else if (snd_byte_cnt == MSG_LEN) { 
    m2_running = false;
    signal_m2_end();
  }
}

void m3step() {
  if (!m3_running){
    return;
  }
  if (m3_steps) {
    if (m3_last_pulse) {
      m3_last_pulse = false;
      digitalWrite(m3_step_pin, false);
      return;
    } else if ((tick_now - m3_tick_last) >= m3_step_interval) {
      digitalWrite(m3_step_pin, true);
      m3_last_pulse = true;
      m3_steps-=m3_finite_mode; //0 for continuous mode, 1 for finite steps
      m3_tick_last = tick_now;
      return;
    }
  } else if (snd_byte_cnt == MSG_LEN) { 
    m3_running = false;
    signal_m3_end();
  }
}

void setup() {
  //Ethernet.begin(mac,ip);
  //Udp.begin(localPort);
  Serial5.setRx(PD2);
  Serial5.setTx(PD3);
  Serial5.begin(115200, SERIAL_8N1);
  
  Serial3.setTx(PC10); // TMC2209 USART, only TX works
  Serial3.begin(115200);
  USART3->CR1 &= ~USART_CR1_RE;  // Disable receiver - STM32 specific
  
  // UCSR0B &= ~_BV(UDRIE0); //disable "USART, Data Register Empty" interrupt, only works on AVR

  //-----MOTOR SETUP-------
  pinMode(m0_enabled_pin, OUTPUT);
  digitalWrite(m0_enabled_pin, HIGH); //disable the motor first

  pinMode(m0_dir_pin, OUTPUT);
  pinMode(m0_step_pin, OUTPUT);

  m0_dir_pin_state = HIGH;
  digitalWrite(m0_dir_pin, HIGH);
  digitalWrite(m0_step_pin, LOW);
  m0_enabled_pin_state = false;
  digitalWrite(m0_enabled_pin, LOW); //finally enable back the motor

  //-----------------m1---------------------
  pinMode(m1_enabled_pin, OUTPUT);
  digitalWrite(m1_enabled_pin, HIGH); //disable the motor first

  pinMode(m1_dir_pin, OUTPUT);
  pinMode(m1_step_pin, OUTPUT);

  m1_dir_pin_state = HIGH;
  digitalWrite(m1_dir_pin, HIGH);
  digitalWrite(m1_step_pin, LOW);
  m1_enabled_pin_state = false;
  digitalWrite(m1_enabled_pin, LOW); //finally enable back the motor

  //-----------------m2---------------------
  pinMode(m2_enabled_pin, OUTPUT);
  digitalWrite(m2_enabled_pin, HIGH); //disable the motor first

  pinMode(m2_dir_pin, OUTPUT);
  pinMode(m2_step_pin, OUTPUT);

  m2_dir_pin_state = HIGH;
  digitalWrite(m2_dir_pin, HIGH);
  digitalWrite(m2_step_pin, LOW);
  m2_enabled_pin_state = false;
  digitalWrite(m2_enabled_pin, LOW); //finally enable back the motor

  //-----------------m3---------------------
  pinMode(m3_enabled_pin, OUTPUT);
  digitalWrite(m3_enabled_pin, HIGH); //disable the motor first

  pinMode(m3_dir_pin, OUTPUT);
  pinMode(m3_step_pin, OUTPUT);

  m3_dir_pin_state = HIGH;
  digitalWrite(m3_dir_pin, HIGH);
  digitalWrite(m3_step_pin, LOW);
  m3_enabled_pin_state = false;
  digitalWrite(m3_enabled_pin, LOW); //finally enable back the motor
  //-----------------------

  TMC2209_Init();

  delay(10);
  tick_now = micros();
  //signal_start();
  //delay(1000);
}

void loop() {
  tick_now = micros();

  //Step the motor if it is running and if delta time has passed
  m0step();
  m1step();
  m2step();
  m3step();

  //Communicate
  process_commands();

}

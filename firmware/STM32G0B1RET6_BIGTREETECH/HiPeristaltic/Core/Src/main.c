/* USER CODE BEGIN Header */
/**

  * Copyright 2025 Gun Deniz Akkoc
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *    http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * https://github.com/gunakkoc/HiPeristaltic
  
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "tmc2209_d.h"
#include <stdbool.h>
#include "stm32g0xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    GPIO_TypeDef *port;  // Pointer to the GPIO port (e.g., GPIOA, GPIOB)
    uint16_t pin;        // Pin number (0 to 15)
} GPIO_Pin;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LEN 256
#define UART_BUFFER_LEN 256
#define MSG_LEN 6
#define USB_INTERMSG_DELAY_US 1300 //minimum 1000us for USB polling + 300us for safety
#define UART_INTERMSG_DELAY_US 366 //(MSGLEN / (115200 * 0.8)) * 1000000 = ~66us + 300us for safety
#define SERIAL_INTERBYTE_TIMEOUT_US 500000
#define MOTOR_MIN_PULSE_WIDTH_US 3 //1us for A4988, 2us for DRV8825, ~100ns for TMC2208 and TMC2209

#define tick_now TIM2->CNT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

USART_HandleTypeDef husart3;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_usart5_rx;
DMA_HandleTypeDef hdma_usart5_tx;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

const uint32_t SUB_US_DIV = 16;
const uint32_t USB_INTERMSG_DELAY = USB_INTERMSG_DELAY_US * SUB_US_DIV;
const uint32_t UART_INTERMSG_DELAY = UART_INTERMSG_DELAY_US * SUB_US_DIV;
const uint32_t SERIAL_INTERBYTE_TIMEOUT = SERIAL_INTERBYTE_TIMEOUT_US * SUB_US_DIV;
const uint32_t MOTOR_MIN_PULSE_WIDTH = MOTOR_MIN_PULSE_WIDTH_US * SUB_US_DIV;
const uint8_t CMD_COUNT = 68;

uint8_t rcv_buffer[BUFFER_LEN];
uint8_t rcv_usb_buffer[BUFFER_LEN];
uint8_t rcv_uart_buffer[UART_BUFFER_LEN];
uint8_t rcv_usb_write_ind = 0;
uint8_t rcv_usb_read_ind = 0;
uint8_t rcv_uart_write_ind = 0;
uint8_t rcv_uart_read_ind = 0;
uint8_t snd_buffer[BUFFER_LEN];
uint8_t rcv_usb_cnt = 0;
uint8_t rcv_uart_cnt = 0;
uint8_t snd_byte_cnt = MSG_LEN + 1;
uint32_t rcv_last_tick = 0;
uint32_t snd_last_tick = 0;
uint8_t checksum = 0;

const uint32_t min2us = 60000000;

uint32_t m0_tick_delta = 0;
uint32_t m1_tick_delta = 0;
uint32_t m2_tick_delta = 0;
uint32_t m3_tick_delta = 0;

// ------- START OF MOTOR PINS AND VARIABLES

// -----------------m0---------------------
const GPIO_Pin m0_enabled_pin = {GPIOB, GPIO_PIN_14};
const GPIO_Pin m0_dir_pin = {GPIOB, GPIO_PIN_12};
const GPIO_Pin m0_step_pin = {GPIOB, GPIO_PIN_13};

bool m0_running = false;
uint32_t m0_tick_last = 0;
uint32_t m0_steps = 0;
uint32_t m0_target_steps = 0;
uint8_t m0_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m0_last_pulse = false;

bool m0_enabled_pin_state = true;
bool m0_dir_pin_state = false;
uint32_t m0_step_interval = 4000;

// -----------------m1---------------------
const GPIO_Pin m1_enabled_pin = {GPIOB, GPIO_PIN_11};
const GPIO_Pin m1_dir_pin = {GPIOB, GPIO_PIN_2};
const GPIO_Pin m1_step_pin = {GPIOB, GPIO_PIN_10};

bool m1_running = false;
uint32_t m1_tick_last = 0;
uint32_t m1_steps = 0;
uint32_t m1_target_steps = 0;
uint8_t m1_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m1_last_pulse = false;

bool m1_enabled_pin_state = true;
bool m1_dir_pin_state = false;
uint32_t m1_step_interval = 4000;

//-----------------m2---------------------
const GPIO_Pin m2_enabled_pin = {GPIOB, GPIO_PIN_1};
const GPIO_Pin m2_dir_pin = {GPIOC, GPIO_PIN_5};
const GPIO_Pin m2_step_pin = {GPIOB, GPIO_PIN_0};

bool m2_running = false;
uint32_t m2_tick_last = 0;
uint32_t m2_steps = 0;
uint32_t m2_target_steps = 0;
uint8_t m2_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m2_last_pulse = false;

bool m2_enabled_pin_state = true;
bool m2_dir_pin_state = false;
uint32_t m2_step_interval = 4000;

//-----------------m3--------------------- //FOR E1: EN D30, DIR D34, STP D36
const GPIO_Pin m3_enabled_pin = {GPIOD, GPIO_PIN_1};
const GPIO_Pin m3_dir_pin = {GPIOB, GPIO_PIN_4};
const GPIO_Pin m3_step_pin = {GPIOB, GPIO_PIN_3};

bool m3_running = false;
uint32_t m3_tick_last = 0;
uint32_t m3_steps = 0;
uint32_t m3_target_steps = 0;
uint8_t m3_finite_mode = 1; //0 for continuous mode, 1 for finite steps
bool m3_last_pulse = false;

bool m3_enabled_pin_state = true;
bool m3_dir_pin_state = false;
uint32_t m3_step_interval = 4000;
// ------- END OF MOTOR PINS AND VARIABLES

//tick_now defined in macro already
//uint32_t tick_now = 0;

// ###################################### Start TMC2209 Variables ######################################
#ifdef TMC2209_driver
extern TMC2209_CONF_t TMC2209_motors[4];
uint8_t TMC2209_ustep_exp_int_temp = 0;
uint32_t TMC2209_ustep_exp_bits_temp = 0;
#endif
// ###################################### END TMC2209 Variables ######################################

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_Init(void);
static void MX_USART5_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_timer_init(void)
{
    // Enable TIM2 Clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Configure TIM2
    TIM_HandleTypeDef htim2;
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (SystemCoreClock / (1000000 * SUB_US_DIV)) - 1; // Set prescaler for 1 MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF; // Max value for 32-bit timer
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        // Initialization Error
        while (1);
    }

    // Start Timer
    HAL_TIM_Base_Start(&htim2);
}

uint32_t micros(void)
{
    return TIM2->CNT; // Read the counter value
}

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
  snd_byte_cnt = 0;
}

void err_checksum(){
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
    m0_last_pulse = false;
	HAL_GPIO_WritePin(m0_step_pin.port, m0_step_pin.pin, false);
    m0_tick_last = tick_now - m0_step_interval;
  }
  send_ack();
}

void get_m0_steps(){
  memcpy(snd_buffer+1,&m0_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_steps(){
  memcpy(&m0_steps,rcv_buffer+1,4);
  m0_target_steps = m0_steps;
  send_ack();
}

void get_m0_target_steps(){
  memcpy(snd_buffer+1,&m0_target_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_target_steps(){
  memcpy(&m0_target_steps,rcv_buffer+1,4);
  send_ack();
}

void get_m0_step_interval(){
  memcpy(snd_buffer+1,&m0_step_interval,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_step_interval(){
  memcpy(&m0_step_interval,rcv_buffer+1,4);
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
  HAL_GPIO_WritePin(m0_dir_pin.port, m0_dir_pin.pin, m0_dir_pin_state);
  send_ack();
}

void get_m0_enabled(){
  snd_buffer[1] = !m0_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_enabled(){
  m0_enabled_pin_state = !rcv_buffer[1];
  HAL_GPIO_WritePin(m0_enabled_pin.port, m0_enabled_pin.pin, m0_enabled_pin_state);
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
    m1_last_pulse = false;
    HAL_GPIO_WritePin(m1_step_pin.port, m1_step_pin.pin, false);
    m1_tick_last = tick_now - m1_step_interval;
  }
  send_ack();
}

void get_m1_steps(){
  memcpy(snd_buffer+1,&m1_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_steps(){
  memcpy(&m1_steps,rcv_buffer+1,4);
  m1_target_steps = m1_steps;
  send_ack();
}

void get_m1_target_steps(){
  memcpy(snd_buffer+1,&m1_target_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_target_steps(){
  memcpy(&m1_target_steps,rcv_buffer+1,4);
  send_ack();
}

void get_m1_step_interval(){
  memcpy(snd_buffer+1,&m1_step_interval,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_step_interval(){
  memcpy(&m1_step_interval,rcv_buffer+1,4);
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
  HAL_GPIO_WritePin(m1_dir_pin.port, m1_dir_pin.pin, m1_dir_pin_state);
  send_ack();
}

void get_m1_enabled(){
  snd_buffer[1] = !m1_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_enabled(){
  m1_enabled_pin_state = !rcv_buffer[1];
  HAL_GPIO_WritePin(m1_enabled_pin.port, m1_enabled_pin.pin, m1_enabled_pin_state);
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
    m2_last_pulse = false;
	HAL_GPIO_WritePin(m2_step_pin.port, m2_step_pin.pin, false);
    m2_tick_last = tick_now - m2_step_interval;
  }
  send_ack();
}

void get_m2_steps(){
  memcpy(snd_buffer+1,&m2_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_steps(){
  memcpy(&m2_steps,rcv_buffer+1,4);
  m2_target_steps = m2_steps;
  send_ack();
}

void get_m2_target_steps(){
  memcpy(snd_buffer+1,&m2_target_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_target_steps(){
  memcpy(&m2_target_steps,rcv_buffer+1,4);
  send_ack();
}

void get_m2_step_interval(){
  memcpy(snd_buffer+1,&m2_step_interval,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_step_interval(){
  memcpy(&m2_step_interval,rcv_buffer+1,4);
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
  HAL_GPIO_WritePin(m2_dir_pin.port, m2_dir_pin.pin, m2_dir_pin_state);
  send_ack();
}

void get_m2_enabled(){
  snd_buffer[1] = !m2_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_enabled(){
  m2_enabled_pin_state = !rcv_buffer[1];
  HAL_GPIO_WritePin(m2_enabled_pin.port, m2_enabled_pin.pin, m2_enabled_pin_state);
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
    m3_last_pulse = false;
	HAL_GPIO_WritePin(m3_step_pin.port, m3_step_pin.pin, false);
    m3_tick_last = tick_now - m3_step_interval;
  }
  send_ack();
}

void get_m3_steps(){
  memcpy(snd_buffer+1,&m3_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_steps(){
  memcpy(&m3_steps,rcv_buffer+1,4);
  m3_target_steps = m3_steps;
  send_ack();
}

void get_m3_target_steps(){
  memcpy(snd_buffer+1,&m3_target_steps,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_target_steps(){
  memcpy(&m3_target_steps,rcv_buffer+1,4);
  send_ack();
}

void get_m3_step_interval(){
  memcpy(snd_buffer+1,&m3_step_interval,4);
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_step_interval(){
  memcpy(&m3_step_interval,rcv_buffer+1,4);
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
  HAL_GPIO_WritePin(m3_dir_pin.port, m3_dir_pin.pin, m3_dir_pin_state);
  send_ack();
}

void get_m3_enabled(){
  snd_buffer[1] = !m3_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_enabled(){
  m3_enabled_pin_state = !rcv_buffer[1];
  HAL_GPIO_WritePin(m3_enabled_pin.port, m3_enabled_pin.pin, m3_enabled_pin_state);
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
  memcpy(snd_buffer+1,&SUB_US_DIV,4);
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


bool process_commands_usb() {
  if (snd_byte_cnt < MSG_LEN){ //data needs sending
	  /*
	  if (snd_byte_cnt){ //if first byte no need delay checking, already flushed
		  if ((tick_now - snd_last_tick) <= USB_INTERMSG_DELAY) { //otherwise need to wait 1ms between transfers
			  return false; //wait if this time hasn't elapsed yet
		  }
	  }
	  CDC_Transmit_FS(&snd_buffer[snd_byte_cnt], 1);
      snd_byte_cnt++;
      */
	  CDC_Transmit_FS(snd_buffer, MSG_LEN);
      snd_last_tick = tick_now;
      snd_byte_cnt = MSG_LEN;
	  return true;
  } else if (snd_byte_cnt == MSG_LEN){ //all data have been sent, now needs flushing
	  if ((tick_now - snd_last_tick) <= USB_INTERMSG_DELAY) { //wait 1ms+ to flush
		  return false; //wait if this time hasn't elapsed yet
	  }
    snd_byte_cnt++; //everything flushed, we can move on
    return true;
  } else if (rcv_usb_cnt == MSG_LEN){ //entire package is received, process
    rcv_usb_cnt = 0;
    if (check_checksum()){
      if (rcv_buffer[0] > CMD_COUNT) {
    	  err_cmd();
    	  return true;
      }
      cmd_fnc_lst[rcv_buffer[0]](); //find the corresponding func by first byte as uint8
    } else {
      err_checksum();
    }
    return true; //continue reading (if any) on next cycle
  } else if (rcv_usb_write_ind - rcv_usb_read_ind){ //if nothing else to do and need reading
    rcv_last_tick = tick_now;
	rcv_buffer[rcv_usb_cnt] = rcv_usb_buffer[rcv_usb_read_ind];
	rcv_usb_read_ind++;
    rcv_usb_cnt++;
    return true;
  } else if ((rcv_usb_cnt) && ((tick_now - rcv_last_tick) > SERIAL_INTERBYTE_TIMEOUT)){ //inter-byte timeout
   	  rcv_usb_read_ind = rcv_usb_write_ind;
   	  rcv_usb_cnt = 0;
   	  return true;
  } else {
    return false; //nothing happened
  }
}

bool process_commands_uart() {
  if (snd_byte_cnt < MSG_LEN){ //data needs sending
	  //we can also send one byte at a time
	  //but with DMA, start sending all at once
	  //this DMA is not circular
	  HAL_UART_Transmit_DMA(&huart5, snd_buffer, MSG_LEN);
      snd_last_tick = tick_now;
      snd_byte_cnt = MSG_LEN;
	  return true;
  } else if (snd_byte_cnt == MSG_LEN){ //all data have been sent, now needs flushing
	  //in case of UART, UART tx completed signal
      return true; //return here to wait next cycle for the signal
  } else if (rcv_uart_cnt == MSG_LEN){ //entire package is received, process
    rcv_uart_cnt = 0;
    if (check_checksum()){
      if (rcv_buffer[0] > CMD_COUNT) {
    	  err_cmd();
    	  return true;
      }
      cmd_fnc_lst[rcv_buffer[0]](); //find the corresponding func by first byte as uint8
    } else {
      err_checksum();
    }
    return true; //continue reading (if any) on next cycle
  } else if (rcv_uart_write_ind - rcv_uart_read_ind){ //if nothing else to do and need reading
	rcv_last_tick = tick_now;
	rcv_buffer[rcv_uart_cnt] = rcv_uart_buffer[rcv_uart_read_ind];
	rcv_uart_read_ind++;
    rcv_uart_cnt++;
    return true;
  } else if ((rcv_uart_cnt) && ((tick_now - rcv_last_tick) > SERIAL_INTERBYTE_TIMEOUT)) {
	  rcv_uart_read_ind = rcv_uart_write_ind;
	  rcv_uart_cnt = 0;
  } else {
    return false; //nothing happened
  }
  return false; //nothing happened
}

// ###################################### Start TMC2209 Functions ######################################
/*
#ifdef TMC2209_driver

//both async mode and UART interrupts needs to be enabled for async receive
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {  // Check if it's the correct UART
        TMC2209_write_ind++; //serves as signal that the transmission is completed.
    }
}

#endif
*/
// ###################################### END TMC2209 Functions ######################################

void m0step() {
  if (!m0_running){
    return;
  }
  if (m0_steps) {
    m0_tick_delta = (tick_now - m0_tick_last);
    if (m0_last_pulse) { //if high switch to low
      if (m0_tick_delta >= MOTOR_MIN_PULSE_WIDTH) { //check for min. motor driver pulse width
        m0_last_pulse = false;
	    HAL_GPIO_WritePin(m0_step_pin.port, m0_step_pin.pin, false);
        // return;
      } //else return;
    } else if (m0_tick_delta >= m0_step_interval) { //if low, check enough time has passed for high
      HAL_GPIO_WritePin(m0_step_pin.port, m0_step_pin.pin, true);
      m0_tick_last = tick_now;
      m0_last_pulse = true;
      m0_steps-=m0_finite_mode; //0 for continuous mode, 1 for finite steps
      // return;
    }
  } else if (m0_last_pulse) {
	  if (m0_tick_delta >= MOTOR_MIN_PULSE_WIDTH) {
		m0_last_pulse = false;
		HAL_GPIO_WritePin(m0_step_pin.port, m0_step_pin.pin, false);
	  }
  } else if (snd_byte_cnt > MSG_LEN) { //if no steps remaining and no message pending
      m0_running = false; //this will prevent reentering here
      signal_m0_end();
      // return;
  }
}

void m1step() {
  if (!m1_running){
    return;
  }
  if (m1_steps) {
    m1_tick_delta = (tick_now - m1_tick_last);
    if (m1_last_pulse) { //if high switch to low
      if (m1_tick_delta >= MOTOR_MIN_PULSE_WIDTH) { //check for min. motor driver pulse width
        m1_last_pulse = false;
	    HAL_GPIO_WritePin(m1_step_pin.port, m1_step_pin.pin, false);
        // return;
      } //else return;
    } else if (m1_tick_delta >= m1_step_interval) { //if low, check enough time has passed for high
      HAL_GPIO_WritePin(m1_step_pin.port, m1_step_pin.pin, true);
      m1_tick_last = tick_now;
      m1_last_pulse = true;
      m1_steps-=m1_finite_mode; //0 for continuous mode, 1 for finite steps
      // return;
    }
  } else if (m1_last_pulse) {
	  if (m1_tick_delta >= MOTOR_MIN_PULSE_WIDTH) {
		m1_last_pulse = false;
		HAL_GPIO_WritePin(m1_step_pin.port, m1_step_pin.pin, false);
	  }
  } else if (snd_byte_cnt > MSG_LEN) { //if no steps remaining and no message pending
      m1_running = false; //this will prevent reentering here
      signal_m1_end();
      // return;
  }
}

void m2step() {
  if (!m2_running){
    return;
  }
  if (m2_steps) {
    m2_tick_delta = (tick_now - m2_tick_last);
    if (m2_last_pulse) { //if high switch to low
      if (m2_tick_delta >= MOTOR_MIN_PULSE_WIDTH) { //check for min. motor driver pulse width
        m2_last_pulse = false;
	    HAL_GPIO_WritePin(m2_step_pin.port, m2_step_pin.pin, false);
        // return;
      } //else return;
    } else if (m2_tick_delta >= m2_step_interval) { //if low, check enough time has passed for high
      HAL_GPIO_WritePin(m2_step_pin.port, m2_step_pin.pin, true);
      m2_tick_last = tick_now;
      m2_last_pulse = true;
      m2_steps-=m2_finite_mode; //0 for continuous mode, 1 for finite steps
      // return;
    }
  } else if (m2_last_pulse) {
	  if (m2_tick_delta >= MOTOR_MIN_PULSE_WIDTH) {
		m2_last_pulse = false;
		HAL_GPIO_WritePin(m2_step_pin.port, m2_step_pin.pin, false);
	  }
  } else if (snd_byte_cnt > MSG_LEN) { //if no steps remaining and no message pending
      m2_running = false; //this will prevent reentering here
      signal_m2_end();
      // return;
  }
}

void m3step() {
  if (!m3_running){
    return;
  }
  if (m3_steps) {
    m3_tick_delta = (tick_now - m3_tick_last);
    if (m3_last_pulse) { //if high switch to low
      if (m3_tick_delta >= MOTOR_MIN_PULSE_WIDTH) { //check for min. motor driver pulse width
        m3_last_pulse = false;
	    HAL_GPIO_WritePin(m3_step_pin.port, m3_step_pin.pin, false);
        // return;
      } //else return;
    } else if (m3_tick_delta >= m3_step_interval) { //if low, check enough time has passed for high
      HAL_GPIO_WritePin(m3_step_pin.port, m3_step_pin.pin, true);
      m3_tick_last = tick_now;
      m3_last_pulse = true;
      m3_steps-=m3_finite_mode; //0 for continuous mode, 1 for finite steps
      // return;
    }
  } else if (m3_last_pulse) {
	  if (m3_tick_delta >= MOTOR_MIN_PULSE_WIDTH) {
		m3_last_pulse = false;
		HAL_GPIO_WritePin(m3_step_pin.port, m3_step_pin.pin, false);
	  }
  } else if (snd_byte_cnt > MSG_LEN) { //if no steps remaining and no message pending
      m3_running = false; //this will prevent reentering here
      signal_m3_end();
      // return;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART5){
		rcv_uart_write_ind = (uint8_t) Size; //here size is the position in the buffer, NOT the number of bytes read
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART5) { //UART serial to PC
	  snd_byte_cnt = MSG_LEN + 1; //values > MSG_LEN means tx completed/flushed
  }
}

void setup() {

  //HAL_UART_Receive_DMA(&huart5, rcv_uart_buffer, MSG_LEN); //for normal DMA
  //HAL_UART_Receive_DMA(&huart5, rcv_uart_buffer, UART_BUFFER_LEN); //never stops with circular DMA
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rcv_uart_buffer, UART_BUFFER_LEN); //triggers rx callback when idle OR buffer filled

  //-----MOTOR SETUP-------

  //-----------------M0---------------------
  HAL_GPIO_WritePin(m0_enabled_pin.port, m0_enabled_pin.pin, true);
  m0_dir_pin_state = true;
  HAL_GPIO_WritePin(m0_dir_pin.port, m0_dir_pin.pin, true);
  HAL_GPIO_WritePin(m0_step_pin.port, m0_step_pin.pin, false);
  m0_enabled_pin_state = false;
  HAL_GPIO_WritePin(m0_enabled_pin.port, m0_enabled_pin.pin, false);

  //-----------------M1---------------------
  HAL_GPIO_WritePin(m1_enabled_pin.port, m1_enabled_pin.pin, true);
  m1_dir_pin_state = true;
  HAL_GPIO_WritePin(m1_dir_pin.port, m1_dir_pin.pin, true);
  HAL_GPIO_WritePin(m1_step_pin.port, m1_step_pin.pin, false);
  m1_enabled_pin_state = false;
  HAL_GPIO_WritePin(m1_enabled_pin.port, m1_enabled_pin.pin, false);

  //-----------------m2---------------------
  HAL_GPIO_WritePin(m2_enabled_pin.port, m2_enabled_pin.pin, true);
  m2_dir_pin_state = true;
  HAL_GPIO_WritePin(m2_dir_pin.port, m2_dir_pin.pin, true);
  HAL_GPIO_WritePin(m2_step_pin.port, m2_step_pin.pin, false);
  m2_enabled_pin_state = false;
  HAL_GPIO_WritePin(m2_enabled_pin.port, m2_enabled_pin.pin, false);

  //-----------------m3---------------------
  HAL_GPIO_WritePin(m3_enabled_pin.port, m3_enabled_pin.pin, true);
  m3_dir_pin_state = true;
  HAL_GPIO_WritePin(m3_dir_pin.port, m3_dir_pin.pin, true);
  HAL_GPIO_WritePin(m3_step_pin.port, m3_step_pin.pin, false);
  m3_enabled_pin_state = false;
  HAL_GPIO_WritePin(m3_enabled_pin.port, m3_enabled_pin.pin, false);
  //-----------------------

  motor_timer_init();

  //tick_now = micros();


#ifdef TMC2209_driver
  TMC2209_Init(husart3);
#endif


  //signal_start();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_TIM2_Init();
  MX_USART3_Init();
  MX_USART5_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);
  setup();
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  while (1){ //UART5 serial (115200)
		//tick_now defined in macro
		//tick_now = micros();
		if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) { //since no VBUS sensing is possible on STM32G0
			HAL_UART_DeInit(&huart5);
			HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
			HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
			snd_byte_cnt = MSG_LEN + 1; //reset the send state, in case mid message.
			break; //as soon as USB is connected, switch to USB serial
			//otherwise stay on UART5
		}
		//Communicate
		process_commands_uart();
		//Step the motor if it is running and if delta time has passed
		m0step();
		m1step();
		m2step();
		m3step();
	  }

	  while (1){ //USB serial
		//tick_now defined in macro
		//tick_now = micros();
		//Communicate
		process_commands_usb();
		//Step the motor if it is running and if delta time has passed
		m0step();
		m1step();
		m2step();
		m3step();
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_LOW;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  husart3.Init.ClockPrescaler = USART_PRESCALER_DIV1;
  husart3.SlaveMode = USART_SLAVEMODE_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetTxFifoThreshold(&husart3, USART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_SetRxFifoThreshold(&husart3, USART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_USARTEx_DisableFifoMode(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

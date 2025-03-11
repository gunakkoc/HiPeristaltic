#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define tick_now time_us_32()

#define BUFFER_LEN 24
#define USB_INTERMSG_DELAY_US 1024
#define SERIAL_INTERBYTE_TIMEOUT_US 500000
#define MOTOR_MIN_PULSE_WIDTH_US 3 //1us for A4988, 2us for DRV8825, ~100ns for TMC2208 and TMC2209

const uint32_t SUB_US_DIV = 1;
const uint32_t USB_INTERMSG_DELAY = USB_INTERMSG_DELAY_US * SUB_US_DIV;
const uint32_t SERIAL_INTERBYTE_TIMEOUT = SERIAL_INTERBYTE_TIMEOUT_US * SUB_US_DIV;
const uint32_t MOTOR_MIN_PULSE_WIDTH = MOTOR_MIN_PULSE_WIDTH_US * SUB_US_DIV;
const uint8_t CMD_COUNT = 68;

const uint8_t MSG_LEN = 6;
uint8_t rcv_buffer[BUFFER_LEN];
uint8_t snd_buffer[BUFFER_LEN];
uint8_t rcv_byte_cnt = 0;
uint8_t snd_byte_cnt = MSG_LEN + 1;
uint32_t rcv_last_tick = 0;
uint32_t snd_last_tick = 0;
uint8_t checksum = 0;
int temp_c;

const uint32_t min2us = 60000000L;

// ------- START OF MOTOR PINS AND VARIABLES
const int m0_enabled_pin = PICO_DEFAULT_LED_PIN; // 38;
const int m0_dir_pin = PICO_DEFAULT_LED_PIN; // A1;
const int m0_step_pin = PICO_DEFAULT_LED_PIN; // A0;

bool m0_running = false;
uint32_t m0_tick_last = 0L;
uint32_t m0_steps = 0L;
uint32_t m0_target_steps = 0L;
uint8_t m0_finite_mode = 1;
bool m0_last_pulse = false;
uint8_t m0_usteps_exp = 0;

bool m0_enabled_pin_state = true;
bool m0_dir_pin_state = false;
uint32_t m0_step_interval = 4000L;

// -----------------m1---------------------
const int m1_enabled_pin = PICO_DEFAULT_LED_PIN; // A2;
const int m1_dir_pin = PICO_DEFAULT_LED_PIN; // A7;
const int m1_step_pin = PICO_DEFAULT_LED_PIN; // A6;

bool m1_running = false;
uint32_t m1_tick_last = 0L;
uint32_t m1_steps = 0L;
uint32_t m1_target_steps = 0L;
uint8_t m1_finite_mode = 1;
bool m1_last_pulse = false;
uint8_t m1_usteps_exp = 0;

bool m1_enabled_pin_state = true;
bool m1_dir_pin_state = false;
uint32_t m1_step_interval = 4000L;

//-----------------m2---------------------
const int m2_enabled_pin = PICO_DEFAULT_LED_PIN; // A8;
const int m2_dir_pin = PICO_DEFAULT_LED_PIN; // 48;
const int m2_step_pin = PICO_DEFAULT_LED_PIN; // 46;

bool m2_running = false;
uint32_t m2_tick_last = 0L;
uint32_t m2_steps = 0L;
uint32_t m2_target_steps = 0L;
uint8_t m2_finite_mode = 1;
bool m2_last_pulse = false;
uint8_t m2_usteps_exp = 0;

bool m2_enabled_pin_state = true;
bool m2_dir_pin_state = false;
uint32_t m2_step_interval = 4000L;

//-----------------m3--------------------- //FOR E1: EN D30, DIR D34, STP D36
const int m3_enabled_pin = PICO_DEFAULT_LED_PIN; // 24; //E0
const int m3_dir_pin = PICO_DEFAULT_LED_PIN; // 28;
const int m3_step_pin = PICO_DEFAULT_LED_PIN; // 26;

bool m3_running = false;
uint32_t m3_tick_last = 0L;
uint32_t m3_steps = 0L;
uint32_t m3_target_steps = 0L;
uint8_t m3_finite_mode = 1;
bool m3_last_pulse = false;
uint8_t m3_usteps_exp = 0;

bool m3_enabled_pin_state = true;
bool m3_dir_pin_state = false;
uint32_t m3_step_interval = 4000L;
// ------- END OF MOTOR PINS AND VARIABLES

bool led_active = false;

uint32_t m0_tick_delta = 0;
uint32_t m1_tick_delta = 0;
uint32_t m2_tick_delta = 0;
uint32_t m3_tick_delta = 0;

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
  led_active = true;
  gpio_put(PICO_DEFAULT_LED_PIN,true);
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
    gpio_put(m0_step_pin, false);
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
  gpio_put(m0_dir_pin,m0_dir_pin_state);
  send_ack();
}

void get_m0_enabled(){
  snd_buffer[1] = !m0_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m0_enabled(){
  m0_enabled_pin_state = !rcv_buffer[1];
  gpio_put(m0_enabled_pin, m0_enabled_pin_state);
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
    gpio_put(m1_step_pin, false);
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
  gpio_put(m1_dir_pin,m1_dir_pin_state);
  send_ack();
}

void get_m1_enabled(){
  snd_buffer[1] = !m1_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m1_enabled(){
  m1_enabled_pin_state = !rcv_buffer[1];
  gpio_put(m1_enabled_pin, m1_enabled_pin_state);
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
    gpio_put(m2_step_pin, false);
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
  gpio_put(m2_dir_pin,m2_dir_pin_state);
  send_ack();
}

void get_m2_enabled(){
  snd_buffer[1] = !m2_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m2_enabled(){
  m2_enabled_pin_state = !rcv_buffer[1];
  gpio_put(m2_enabled_pin, m2_enabled_pin_state);
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
    gpio_put(m3_step_pin, false);
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
  gpio_put(m3_dir_pin,m3_dir_pin_state);
  send_ack();
}

void get_m3_enabled(){
  snd_buffer[1] = !m3_enabled_pin_state;
  snd_buffer[0] = rcv_buffer[0];
  send_buffer();
}

void set_m3_enabled(){
  m3_enabled_pin_state = !rcv_buffer[1];
  gpio_put(m3_enabled_pin, m3_enabled_pin_state);
  send_ack();
}

//variable microstepping support, not supported by default
//either implement by switching resolution pins or by implementing commands to TMC motor driver via UART
void get_m0_var_ustep_support(){
    snd_buffer[1] = false;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void get_m1_var_ustep_support(){
    snd_buffer[1] = false;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void get_m2_var_ustep_support(){
    snd_buffer[1] = false;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void get_m3_var_ustep_support(){
    snd_buffer[1] = false;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

//exponents of microstepping, microstepping = 1/(2^exponent) steps
void get_m0_usteps_exp(){
    snd_buffer[1] = m0_usteps_exp;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void set_m0_usteps_exp(){
    err_cmd(); //not implemented
}

void get_m1_usteps_exp(){
    snd_buffer[1] = m1_usteps_exp;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void set_m1_usteps_exp(){
  err_cmd(); //not implemented
}

void get_m2_usteps_exp(){
    snd_buffer[1] = m2_usteps_exp;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void set_m2_usteps_exp(){
  err_cmd(); //not implemented
}

void get_m3_usteps_exp(){
    snd_buffer[1] = m3_usteps_exp;
    snd_buffer[0] = rcv_buffer[0];
    send_buffer();
}

void set_m3_usteps_exp(){
  err_cmd(); //not implemented
}

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

bool process_commands_usb() {
  if (snd_byte_cnt < MSG_LEN){ //data needs sending
    if (snd_byte_cnt){ //if first byte no need delay checking, already flushed
		  if ((tick_now - snd_last_tick) <= USB_INTERMSG_DELAY) { //otherwise need to wait 1ms between transfers
			  return false; //wait if this time hasn't elapsed yet
		  }
	  }
    putchar(snd_buffer[snd_byte_cnt]);
    snd_last_tick = tick_now;
    snd_byte_cnt++;
    return true;
  } else if (snd_byte_cnt == MSG_LEN){ //entire package is sent
    if ((tick_now - snd_last_tick) <= USB_INTERMSG_DELAY) { //wait 1ms+ to flush
		  return false; //wait if this time hasn't elapsed yet
	  }
    snd_byte_cnt++;
    return true;
  } else if (rcv_byte_cnt == MSG_LEN){ //entire package is received, process
    rcv_byte_cnt = 0;
    if (check_checksum()){
      if (rcv_buffer[0] > CMD_COUNT) { //invalid command
        err_cmd();
        return true;
      }
      cmd_fnc_lst[rcv_buffer[0]](); //find the corresponding func by first byte as uint8
      return true;
    } else {
      err_checksum(); //request data again
    }
    return true; //continue reading (if any) on next cycle
  }
  temp_c = getchar_timeout_us(0);
  if (temp_c == PICO_ERROR_TIMEOUT) { //no data to be read
    if (rcv_byte_cnt) { //if not the first byte
      if ((tick_now - rcv_last_tick) > SERIAL_INTERBYTE_TIMEOUT) { //handle interbyte timeout
        rcv_byte_cnt = 0;
        return true;
      } else {
        return false; //no data to be read, no timeout return false
      }
    }
  } else { //data was availabe and read to temp_c
    rcv_last_tick = tick_now;
    rcv_buffer[rcv_byte_cnt] = (uint8_t) temp_c;
    rcv_byte_cnt++;
    return true;
  }
  return false;
}

bool process_commands_uart() {
    if (snd_byte_cnt < MSG_LEN){ //data needs sending
        if (!uart_is_writable(UART_ID)) {
            return false;
        }
        uart_putc_raw(UART_ID, snd_buffer[snd_byte_cnt]);
        snd_byte_cnt++;
        return true;
    } else if (snd_byte_cnt == MSG_LEN){ //entire package is sent
        if (!uart_is_writable(UART_ID)) {
            return false;
        }
        snd_byte_cnt++;
        return true;
    } else if (rcv_byte_cnt == MSG_LEN){ //entire package is received, process
        rcv_byte_cnt = 0;
        if (check_checksum()){
            if (rcv_buffer[0] > CMD_COUNT) { //invalid command
                err_cmd();
                return true;
            }
            cmd_fnc_lst[rcv_buffer[0]](); //find the corresponding func by first byte as uint8
            return true;
        } else {
            err_checksum(); //request data again
        }
        return true; //continue reading (if any) on next cycle
    } else if (rcv_byte_cnt) { //if not the first byte
        if ((tick_now - rcv_last_tick) > SERIAL_INTERBYTE_TIMEOUT) { //check interbyte timeout
            rcv_byte_cnt = 0; //reset if timeout
            return true;
        }
    } else if (uart_is_readable(UART_ID)) {
        rcv_buffer[rcv_byte_cnt] = uart_getc(UART_ID);
        rcv_byte_cnt++;
        rcv_last_tick = tick_now;
        return true;
    }
    return false;
}

void led_flip(){
  led_active = !led_active;
  gpio_put(PICO_DEFAULT_LED_PIN,led_active);
}

void m0step() {
  if (!m0_running){
    return;
  }
  if (m0_steps) {
    m0_tick_delta = (tick_now - m0_tick_last);
    if (m0_last_pulse) { //if high switch to low
      if (m0_tick_delta >= MOTOR_MIN_PULSE_WIDTH) { //check for min. motor driver pulse width
        m0_last_pulse = false;
        gpio_put(m0_step_pin, false);
        // return;
      } //else return;
    } else if (m0_tick_delta >= m0_step_interval) { //if low, check enough time has passed for high
      gpio_put(m0_step_pin, true);
      m0_last_pulse = true;
      m0_steps-=m0_finite_mode;
      m0_tick_last = tick_now;
      // return;
    }
  } else if (m0_last_pulse) {
    m0_tick_delta = (tick_now - m0_tick_last);
    if (m0_tick_delta >= MOTOR_MIN_PULSE_WIDTH)  {
      m0_last_pulse = false;
      gpio_put(m0_step_pin, false);
      // return;
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
        gpio_put(m1_step_pin, false);
        // return;
      } //else return;
    } else if (m1_tick_delta >= m1_step_interval) { //if low, check enough time has passed for high
      gpio_put(m1_step_pin, true);
      m1_last_pulse = true;
      m1_steps-=m1_finite_mode;
      m1_tick_last = tick_now;
      // return;
    }
  } else if (m1_last_pulse) {
    m1_tick_delta = (tick_now - m1_tick_last);
    if (m1_tick_delta >= MOTOR_MIN_PULSE_WIDTH)  {
      m1_last_pulse = false;
      gpio_put(m1_step_pin, false);
      // return;
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
        gpio_put(m2_step_pin, false);
        // return;
      } //else return;
    } else if (m2_tick_delta >= m2_step_interval) { //if low, check enough time has passed for high
      gpio_put(m2_step_pin, true);
      m2_last_pulse = true;
      m2_steps-=m2_finite_mode;
      m2_tick_last = tick_now;
      // return;
    }
  } else if (m2_last_pulse) {
    m2_tick_delta = (tick_now - m2_tick_last);
    if (m2_tick_delta >= MOTOR_MIN_PULSE_WIDTH)  {
      m2_last_pulse = false;
      gpio_put(m2_step_pin, false);
      // return;
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
        gpio_put(m3_step_pin, false);
        // return;
      } //else return;
    } else if (m3_tick_delta >= m3_step_interval) { //if low, check enough time has passed for high
      gpio_put(m3_step_pin, true);
      m3_last_pulse = true;
      m3_steps-=m3_finite_mode;
      m3_tick_last = tick_now;
      // return;
    }
  } else if (m3_last_pulse) {
    m3_tick_delta = (tick_now - m3_tick_last);
    if (m3_tick_delta >= MOTOR_MIN_PULSE_WIDTH)  {
      m3_last_pulse = false;
      gpio_put(m3_step_pin, false);
      // return;
    }
  } else if (snd_byte_cnt > MSG_LEN) { //if no steps remaining and no message pending
      m3_running = false; //this will prevent reentering here
      signal_m3_end();
      // return;
  }
}

void setup() {
  stdio_init_all();
  stdio_set_translate_crlf(&stdio_usb, false);

  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_fifo_enabled(UART_ID, false); //disable UART TX fifo, as we will do non-blocking single byte writes

  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN,false);
  led_active = false;

  //-----MOTOR SETUP-------
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m0_enabled_pin, GPIO_OUT);
  gpio_put(m0_enabled_pin, true); //disable the motor first

  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m0_dir_pin, GPIO_OUT);
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m0_step_pin, GPIO_OUT);

  m0_dir_pin_state = true;
  gpio_put(m0_dir_pin, true);
  gpio_put(m0_step_pin, false);
  m0_enabled_pin_state = false;
  gpio_put(m0_enabled_pin, false); //finally enable back the motor

  //-----------------m1---------------------
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m1_enabled_pin, GPIO_OUT);
  gpio_put(m1_enabled_pin, true); //disable the motor first

  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m1_dir_pin, GPIO_OUT);
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m1_step_pin, GPIO_OUT);

  m1_dir_pin_state = true;
  gpio_put(m1_dir_pin, true);
  gpio_put(m1_step_pin, false);
  m1_enabled_pin_state = false;
  gpio_put(m1_enabled_pin, false); //finally enable back the motor

  //-----------------m2---------------------
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m2_enabled_pin, GPIO_OUT);
  gpio_put(m2_enabled_pin, true); //disable the motor first

  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m2_dir_pin, GPIO_OUT);
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m2_step_pin, GPIO_OUT);

  m2_dir_pin_state = true;
  gpio_put(m2_dir_pin, true);
  gpio_put(m2_step_pin, false);
  m2_enabled_pin_state = false;
  gpio_put(m2_enabled_pin, false); //finally enable back the motor

  //-----------------m3---------------------
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m3_enabled_pin, GPIO_OUT);
  gpio_put(m3_enabled_pin, true); //disable the motor first

  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m3_dir_pin, GPIO_OUT);
  gpio_init(PICO_DEFAULT_LED_PIN); gpio_set_dir(m3_step_pin, GPIO_OUT);

  m3_dir_pin_state = true;
  gpio_put(m3_dir_pin, true);
  gpio_put(m3_step_pin, false);
  m3_enabled_pin_state = false;
  gpio_put(m3_enabled_pin, false); //finally enable back the motor
  //-----------------------

  sleep_ms(50);
  //signal_start();
  //sleep_ms(10);
}

void main() {
  setup();

  while(true){

    while(true){
        if (stdio_usb_connected()) {
            uart_deinit(UART_ID);
            rcv_byte_cnt = 0;
            snd_byte_cnt = MSG_LEN + 1;
            sleep_ms(200); //wait for USB to be ready
            break; //swicth to USB communication until restart
        }

        //Step the motor if it is running and if delta time has passed
        m0step();
        m1step();
        m2step();
        m3step();

        //Communicate over UART while USB is NOT connected
        process_commands_uart();
    }

    while(true){
        //Step the motor if it is running and if delta time has passed
        m0step();
        m1step();
        m2step();
        m3step();

        //Communicate over USB while USB is connected
        process_commands_usb();
    }

  }
}

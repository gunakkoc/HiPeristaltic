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

HardwareSerial Serial5(USART5);

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

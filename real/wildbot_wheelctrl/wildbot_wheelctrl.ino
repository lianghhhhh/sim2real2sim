#include <esp_system.h>

#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;

// device settings.
#define WHEEL_RX 18
#define WHEEL_TX 19

// USB serial to the host. Must match `wheel_baudrate` in the host's hardware.yaml.
#define SERIAL_BAUDRATE 460800
#define WHEEL_BAUDRATE 115200

#define TIME_BETWEEN_CMD 4

// wheel IDs on the bus are 1..NUM_WHEELS.
#define NUM_WHEELS 4

// polling mode: one cycle sends the latest target to every wheel and reads
// its reply, one wheel per slot. The slot is the same spacing stop_all_wheels()
// has always used, so the bus timing does not change.
#define POLL_CYCLE_US 20000UL        // 50 Hz per wheel
#define POLL_SLOT_US 4000UL
#define POLL_REPLY_TIMEOUT_US 3500UL

// uncomment to dump every raw wheel packet as HEX. Debugging only: the extra
// lines roughly double the USB traffic and are not JSON.
// #define DEBUG_PRINT_PACKET

const size_t packet_length = 10;
uint8_t packet_move[packet_length] = {0x01, 0x64, 0xff, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 0xda};

// -1: off
// 500: wheel stops when there is no new cmd received in the past 500ms.
int heartbeat_time_ms = 500;
unsigned long prev_time = millis();
bool stop_flag = false;

bool get_info_flag = false;

// polling mode state. See wheel_poll.h.
bool poll_enabled = false;
int16_t poll_target[NUM_WHEELS] = {0, 0, 0, 0};
uint8_t poll_act = 0;
uint8_t poll_slot = 0;               // 0..NUM_WHEELS-1: wheel slots, NUM_WHEELS: idle gap
bool poll_waiting = false;
unsigned long poll_cycle_start_us = 0;
unsigned long poll_sent_us = 0;
uint8_t poll_ok_mask = 0;            // bit i: wheel i+1 answered in this cycle
uint8_t poll_pending_mode_id = 0;    // 0: nothing queued
uint8_t poll_pending_mode = 0;

// latest feedback of each wheel, kept across cycles.
uint8_t fb_mode[NUM_WHEELS] = {0, 0, 0, 0};
int16_t fb_tor[NUM_WHEELS] = {0, 0, 0, 0};
int16_t fb_spd[NUM_WHEELS] = {0, 0, 0, 0};
uint16_t fb_pos[NUM_WHEELS] = {0, 0, 0, 0};
uint8_t fb_err[NUM_WHEELS] = {0, 0, 0, 0};


// func to print the packet_move data as HEX.
void print_packet(uint8_t *packet, size_t length) {
#ifdef DEBUG_PRINT_PACKET
  for (size_t i = 0; i < length; i++) {
    if (i > 0) Serial.print(", ");
    Serial.print("0x");
    if (packet[i] < 0x10) Serial.print("0");
    Serial.print(packet[i], HEX);
  }
  Serial.println();
#endif
}


// CRC-8/MAXIM
uint8_t crc8_update(uint8_t crc, uint8_t data) {
  uint8_t i;
  crc = crc ^ data;
  for (i = 0; i < 8; ++i) {
    if (crc & 0x01) {
      crc = (crc >> 1) ^ 0x8c;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}


// clear wheel serial buffer
void clear_wheel_buffer() {
  while (Serial1.available() > 0) {
    Serial1.read();
  }
}


// current loop, cmd: -32767 ~ 32767 -> -8 ~ 8 A (max current < 2.7A)
// speed loop, cmd: -200 ~ 200 rpm
// position loop, cmd: 0 ~ 32767 -> 0 ~ 360°

//    wherever the mode is set to position mode
//    the currently position is the 0 position and it moves to the goal position
//    at the direction as the shortest path.
void wheel_ctrl(uint8_t id, int cmd, uint8_t act) {
  packet_move[0] = id;
  packet_move[1] = 0x64;

  packet_move[2] = (cmd >> 8) & 0xFF;
  packet_move[3] = cmd & 0xFF;

  packet_move[4] = 0x00;
  packet_move[5] = 0x00;

  packet_move[6] = act;
  packet_move[7] = 0x00;

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;

  Serial1.write(packet_move, packet_length);
}


// change mode
// 1 - current loop
// 2 - speed loop
// 3 - position loop

void wheel_change_mode(uint8_t id, uint8_t mode) {
  packet_move[0] = id;
  packet_move[1] = 0xA0;
  packet_move[2] = 0x00;
  packet_move[3] = 0x00;
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;
  packet_move[8] = 0x00;
  packet_move[9] = mode;
  Serial1.write(packet_move, packet_length);
  print_packet(packet_move, packet_length);
}


// get info
// feedback:
// 0  1    2        3        4       5       6    7  8     9
// ID MODE TORQUE_H TORQUE_L SPEED_H SPEED_L TEMP U8 ERROR CRC8
// U8 is the position as 0~255 -> 0~360°, not a voltage.
void wheel_get_info(uint8_t id) {
  packet_move[0] = id;

  get_info_flag = true;

  packet_move[1] = 0x74;

  packet_move[2] = 0x00;
  packet_move[3] = 0x00;

  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;

  packet_move[8] = 0x00;
  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;
  Serial1.write(packet_move, packet_length);
}


// stop a single wheel.
void wheel_stop(uint8_t id) {
  wheel_ctrl(id, 0, 0);
}


void stop_all_wheels() {
  wheel_stop(1);
  delay(TIME_BETWEEN_CMD);
  wheel_stop(2);
  delay(TIME_BETWEEN_CMD);
  wheel_stop(3);
  delay(TIME_BETWEEN_CMD);
  wheel_stop(4);
  delay(TIME_BETWEEN_CMD);
}


// set the heartbeat time.
void set_heartbeat_time(int time_ms) {
  heartbeat_time_ms = time_ms;
}


// heartbeat ctrl
void heartbeat_ctrl() {
  if (heartbeat_time_ms == -1) {
    return;
  }
  unsigned long curr_time = millis();
  if (curr_time - prev_time > (unsigned long)heartbeat_time_ms && !stop_flag) {
    if (poll_enabled) {
      // the poll cycle keeps running, so the wheels are told to stop on the
      // next cycle and the host still gets feedback.
      for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        poll_target[i] = 0;
      }
    } else {
      stop_all_wheels();
    }
    stop_flag = true;
    Serial.println("{\"T\":20011,\"hb\":0}");
  }
}


// json cmds.
#include "json_cmd.h"

// esp32-side polling of all wheels.
#include "wheel_poll.h"

// uart ctrl funcs.
#include "uart_ctrl.h"


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial1.begin(WHEEL_BAUDRATE, SERIAL_8N1, WHEEL_RX, WHEEL_TX);

  // clear wheel buffer.
  clear_wheel_buffer();

  // Ensure the controller starts from a stopped state.
  stop_all_wheels();
}


void loop() {
  // heartbeat function.
  heartbeat_ctrl();

  if (poll_enabled) {
    // the poll cycle sends the targets and reads the replies.
    poll_ctrl();
  } else {
    // recving data from wheel.
    wheel_fb();
  }

  // recving the json cmd from uart.
  serialCtrl();
}

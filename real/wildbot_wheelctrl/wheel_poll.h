// ESP32-side polling of all wheels.
//
// The RS-485 bus is half duplex and the motors answer every command, so two
// commands sent too close together collide with a reply. While the host sent
// one wheel at a time, its own 10 ms gap was the only thing preventing that,
// and USB buffering can deliver two commands back to back.
//
// In polling mode the ESP32 owns the bus timing instead. The host only updates
// the targets (CMD_WHEEL_CTRL_ALL); every POLL_CYCLE_US the ESP32 sends the
// latest target to wheel 1..NUM_WHEELS, one per POLL_SLOT_US, reads each
// reply, then reports all wheels in one FB_MOTOR_ALL line.

void poll_start() {
  poll_slot = 0;
  poll_waiting = false;
  poll_ok_mask = 0;
  poll_cycle_start_us = micros();
  poll_enabled = true;
}


void poll_stop() {
  poll_enabled = false;
  // let an in-flight reply finish before the bus is used directly again.
  delay(TIME_BETWEEN_CMD);
  clear_wheel_buffer();
}


// {"T":20012,"ms":123456,"ok":15,"mode":[2,2,2,2],"spd":[50,50,-50,-50],
//  "pos":[1234,22222,333,4444],"tor":[120,118,-121,-119],"err":[0,0,0,0]}
// arrays are indexed by wheel id - 1. A wheel whose bit in "ok" is 0 did not
// answer in this cycle and its entries are the last values it reported.
// wheel i is sampled (NUM_WHEELS - i) * POLL_SLOT_US before "ms", roughly.
void poll_send_frame() {
  // longest possible line is 172 chars.
  char buf[224];
  snprintf(buf, sizeof(buf),
           "{\"T\":%d,\"ms\":%lu,\"ok\":%u,"
           "\"mode\":[%u,%u,%u,%u],"
           "\"spd\":[%d,%d,%d,%d],"
           "\"pos\":[%u,%u,%u,%u],"
           "\"tor\":[%d,%d,%d,%d],"
           "\"err\":[%u,%u,%u,%u]}",
           FB_MOTOR_ALL, millis(), poll_ok_mask,
           fb_mode[0], fb_mode[1], fb_mode[2], fb_mode[3],
           fb_spd[0], fb_spd[1], fb_spd[2], fb_spd[3],
           fb_pos[0], fb_pos[1], fb_pos[2], fb_pos[3],
           fb_tor[0], fb_tor[1], fb_tor[2], fb_tor[3],
           fb_err[0], fb_err[1], fb_err[2], fb_err[3]);
  Serial.println(buf);
}


// reads the reply of the wheel in the current slot.
// 0  1    2        3        4       5       6     7     8     9
// ID MODE TORQUE_H TORQUE_L SPEED_H SPEED_L POS_H POS_L ERROR CRC8
void poll_read_reply() {
  uint8_t data[packet_length];
  Serial1.readBytes(data, packet_length);
  print_packet(data, packet_length);

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9] || data[0] != poll_slot + 1) {
    return;
  }

  fb_mode[poll_slot] = data[1];
  fb_tor[poll_slot] = (int16_t)((data[2] << 8) | data[3]);
  fb_spd[poll_slot] = (int16_t)((data[4] << 8) | data[5]);
  fb_pos[poll_slot] = (uint16_t)((data[6] << 8) | data[7]);
  fb_err[poll_slot] = data[8];
  poll_ok_mask |= (1 << poll_slot);
}


void poll_ctrl() {
  if (poll_slot < NUM_WHEELS) {
    if (!poll_waiting) {
      // slot k starts k * POLL_SLOT_US after the cycle start.
      if (micros() - poll_cycle_start_us >= (unsigned long)poll_slot * POLL_SLOT_US) {
        clear_wheel_buffer();
        wheel_ctrl(poll_slot + 1, poll_target[poll_slot], poll_act);
        poll_sent_us = micros();
        poll_waiting = true;
      }
      return;
    }

    if (Serial1.available() >= (int)packet_length) {
      poll_read_reply();
    } else if (micros() - poll_sent_us < POLL_REPLY_TIMEOUT_US) {
      return;
    }

    poll_waiting = false;
    poll_slot++;
    if (poll_slot == NUM_WHEELS) {
      poll_send_frame();
    }
    return;
  }

  // idle gap until the next cycle. A mode change has no reply, so it is sent
  // here where it cannot collide with one.
  if (poll_pending_mode_id != 0) {
    wheel_change_mode(poll_pending_mode_id, poll_pending_mode);
    poll_pending_mode_id = 0;
  }

  unsigned long elapsed = micros() - poll_cycle_start_us;
  if (elapsed >= POLL_CYCLE_US) {
    // keep a fixed rate, but resync after falling a whole cycle behind.
    if (elapsed >= 2 * POLL_CYCLE_US) {
      poll_cycle_start_us = micros();
    } else {
      poll_cycle_start_us += POLL_CYCLE_US;
    }
    poll_slot = 0;
    poll_ok_mask = 0;
  }
}

// true for a wheel id that polling mode handles.
bool poll_id_ok(int id) {
  return id >= 1 && id <= NUM_WHEELS;
}


// In polling mode the poll cycle owns the bus, so single-wheel commands only
// update the targets and are sent on the next cycle.
void jsonCmdReceiveHandler(){
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch(cmdType){
	case CMD_WHEEL_STOP: {
                int id = jsonCmdReceive["id"];
                if (!poll_enabled) {
                  wheel_stop(id);
                } else if (poll_id_ok(id)) {
                  poll_target[id - 1] = 0;
                }
                break;
  }
	case CMD_WHEEL_CTRL: {
                int id = jsonCmdReceive["id"];
                if (!poll_enabled) {
                  wheel_ctrl(
                  id,
                  jsonCmdReceive["cmd"],
                  jsonCmdReceive["act"]);
                } else if (poll_id_ok(id)) {
                  poll_target[id - 1] = constrain(jsonCmdReceive["cmd"].as<int>(), -32767, 32767);
                  poll_act = jsonCmdReceive["act"];
                }
                break;
  }
  case CMD_WHEEL_CTRL_ALL: {
                JsonArray cmd = jsonCmdReceive["cmd"].as<JsonArray>();
                if (cmd.size() != NUM_WHEELS) {
                  break;
                }
                for (uint8_t i = 0; i < NUM_WHEELS; i++) {
                  poll_target[i] = constrain(cmd[i].as<int>(), -32767, 32767);
                }
                poll_act = jsonCmdReceive["act"] | 0;
                if (!poll_enabled) {
                  poll_start();
                }
                break;
  }
  case CMD_WHEEL_CHANGE_MODE: {
                int id = jsonCmdReceive["id"];
                if (!poll_enabled) {
                  wheel_change_mode(
                  id,
                  jsonCmdReceive["mode"]);
                } else if (poll_id_ok(id)) {
                  // sent in the idle gap of the poll cycle.
                  poll_pending_mode_id = id;
                  poll_pending_mode = jsonCmdReceive["mode"];
                }
                break;
  }
  case CMD_WHEEL_POLL: {
                int en = jsonCmdReceive["en"] | 0;
                if (en && !poll_enabled) {
                  poll_start();
                } else if (!en && poll_enabled) {
                  poll_stop();
                  stop_all_wheels();
                }
                break;
  }
  case CMD_WHEEL_INFO:
                // the info reply has a different layout, so it is only
                // available outside polling mode.
                if (!poll_enabled) {
                  wheel_get_info(
                  jsonCmdReceive["id"]);
                }
                break;
	case CMD_HEARTBEAT_TIME:
                set_heartbeat_time(
								jsonCmdReceive["time"]);break;


  // esp-32 dev ctrl.
  case CMD_REBOOT:      esp_restart();break;
	}
}

void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      // Now we have received the complete JSON string
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
      	prev_time = millis();
      	if (stop_flag) {
      		stop_flag = false;
      	}
      	// in polling mode a reply may be on its way in; the poll cycle
      	// clears the buffer itself before each command.
      	if (!poll_enabled) {
      		clear_wheel_buffer();
      	}
        jsonCmdReceiveHandler();
      } else {
        // Handle JSON parsing error here
      }
      // Reset the receivedData for the next JSON string
      receivedData = "";
    } else if (receivedData.length() > 512) {
      // no command is this long: it is noise (e.g. a baud rate mismatch).
      receivedData = "";
    }
  }
}


void wheel_fb() {
  if (Serial1.available() >= 10) {
    uint8_t data[10];
    Serial1.readBytes(data, 10);

    uint8_t wheel_id = data[0];

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
      crc = crc8_update(crc, data[i]);
    }
    if (crc != data[9]){
      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["crc"] = 0;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      return;
    }

    int wheel_mode = data[1];

    int wheel_torque = (data[2] << 8) | data[3];
    if (wheel_torque & 0x8000) {
      wheel_torque = -(0x10000 - wheel_torque);
    }

    int wheel_spd = (data[4] << 8) | data[5];
    if (wheel_spd & 0x8000) {
      wheel_spd = -(0x10000 - wheel_spd);
    }

    if (get_info_flag) {
      get_info_flag = false;
      int wheel_temp = data[6];
      int wheel_u8 = data[7];

      int wheel_error = data[8];

      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["id"] = wheel_id;
      jsonInfoSend["mode"] = wheel_mode;
      jsonInfoSend["tor"] = wheel_torque;
      jsonInfoSend["spd"] = wheel_spd;
      jsonInfoSend["temp"] = wheel_temp;
      jsonInfoSend["u8"] = wheel_u8;
      jsonInfoSend["err"] = wheel_error;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      print_packet(data, 10);
    } else {
      int wheel_pos = (data[6] << 8) | data[7];
      // if (wheel_pos & 0x8000) {
      //   wheel_pos = -(0x10000 - wheel_pos);
      // }

      int wheel_error = data[8];

      jsonInfoSend.clear();
      jsonInfoSend["T"] = FB_MOTOR;
      jsonInfoSend["id"] = wheel_id;
      jsonInfoSend["mode"] = wheel_mode;
      jsonInfoSend["tor"] = wheel_torque;
      jsonInfoSend["spd"] = wheel_spd;
      jsonInfoSend["pos"] = wheel_pos;
      jsonInfoSend["err"] = wheel_error;
      String getInfoJsonString;
      serializeJson(jsonInfoSend, getInfoJsonString);
      Serial.println(getInfoJsonString);
      print_packet(data, 10);
    }
  }
}



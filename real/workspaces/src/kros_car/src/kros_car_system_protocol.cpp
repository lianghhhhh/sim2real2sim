#include "kros_car/kros_car_system.hpp"

#include <algorithm>
#include <chrono>

namespace kros_car
{

std::vector<uint8_t> KrosCarSystem::arm_build_packet(
  uint8_t servo_id, uint8_t cmd, const std::vector<uint8_t> & params)
{
  uint8_t length = static_cast<uint8_t>(params.size() + 3);
  std::vector<uint8_t> body;
  body.push_back(servo_id);
  body.push_back(length);
  body.push_back(cmd);
  body.insert(body.end(), params.begin(), params.end());

  uint8_t sum = 0;
  for (auto b : body) {
    sum += b;
  }
  uint8_t checksum = (~sum) & 0xFF;

  std::vector<uint8_t> frame;
  frame.push_back(0x55);
  frame.push_back(0x55);
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(checksum);
  return frame;
}

std::optional<ArmServoPacket> KrosCarSystem::arm_read_packet(int timeout_ms)
{
  // arm_serial_mutex_ must be held by caller
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      uint8_t byte;
      arm_serial_.ReadByte(byte, 1);  // 1ms timeout per byte attempt
      if (byte != 0x55) {
        continue;
      }

      arm_serial_.ReadByte(byte, 1);
      if (byte != 0x55) {
        continue;
      }

      uint8_t servo_id, length;
      arm_serial_.ReadByte(servo_id, 1);
      arm_serial_.ReadByte(length, 1);

      if (length < 3) {
        continue;
      }

      // Protocol length includes [id, length, cmd, params..., checksum], so
      // bytes remaining after reading id+length are (length - 1).
      const size_t remaining = static_cast<size_t>(length - 1);
      std::vector<uint8_t> payload(remaining);
      for (size_t i = 0; i < remaining; ++i) {
        arm_serial_.ReadByte(payload[i], 1);
      }

      uint8_t cmd = payload[0];
      std::vector<uint8_t> params(payload.begin() + 1, payload.end() - 1);
      uint8_t recv_checksum = payload.back();

      // Verify checksum
      uint8_t sum = servo_id + length + cmd;
      for (auto p : params) {
        sum += p;
      }
      uint8_t calc_checksum = (~sum) & 0xFF;

      if (recv_checksum != calc_checksum) {
        continue;
      }

      return ArmServoPacket{servo_id, length, cmd, params};
    } catch (const LibSerial::ReadTimeout &) {
      continue;
    } catch (...) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

bool KrosCarSystem::arm_move_time_write(uint8_t servo_id, int position_ticks, int time_ms)
{
  position_ticks = std::clamp(position_ticks, 0, 1000);
  time_ms = std::clamp(time_ms, 0, 30000);

  uint8_t pos_l = static_cast<uint8_t>(position_ticks & 0xFF);
  uint8_t pos_h = static_cast<uint8_t>((position_ticks >> 8) & 0xFF);
  uint8_t time_l = static_cast<uint8_t>(time_ms & 0xFF);
  uint8_t time_h = static_cast<uint8_t>((time_ms >> 8) & 0xFF);

  auto frame = arm_build_packet(servo_id, 1, {pos_l, pos_h, time_l, time_h});

  try {
    arm_serial_.Write(frame);
    return true;
  } catch (...) {
    return false;
  }
}

bool KrosCarSystem::arm_move_stop(uint8_t servo_id)
{
  auto frame = arm_build_packet(servo_id, 12, {});
  try {
    arm_serial_.Write(frame);
    return true;
  } catch (...) {
    return false;
  }
}

std::optional<int> KrosCarSystem::arm_read_position(uint8_t servo_id, int timeout_ms)
{
  // arm_serial_mutex_ must be held by caller
  auto frame = arm_build_packet(servo_id, 28, {});
  try {
    arm_serial_.FlushInputBuffer();
    arm_serial_.Write(frame);
  } catch (...) {
    return std::nullopt;
  }

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    int remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    auto packet = arm_read_packet(std::max(1, remaining));
    if (!packet) {
      continue;
    }
    if (packet->servo_id != servo_id || packet->cmd != 28) {
      continue;
    }
    if (packet->params.size() < 2) {
      continue;
    }
    int value = packet->params[0] | (packet->params[1] << 8);
    if (value >= 0x8000) {
      value -= 0x10000;
    }
    return value;
  }
  return std::nullopt;
}

std::optional<int> KrosCarSystem::arm_read_temperature(uint8_t servo_id, int timeout_ms)
{
  // arm_serial_mutex_ must be held by caller
  auto frame = arm_build_packet(servo_id, 26, {});
  try {
    arm_serial_.FlushInputBuffer();
    arm_serial_.Write(frame);
  } catch (...) {
    return std::nullopt;
  }

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    int remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    auto packet = arm_read_packet(std::max(1, remaining));
    if (!packet) {
      continue;
    }
    if (packet->servo_id != servo_id || packet->cmd != 26) {
      continue;
    }
    if (packet->params.empty()) {
      continue;
    }
    return static_cast<int>(packet->params[0]);
  }
  return std::nullopt;
}

bool KrosCarSystem::arm_set_torque(uint8_t servo_id, bool enable)
{
  auto frame = arm_build_packet(servo_id, 31, {static_cast<uint8_t>(enable ? 1 : 0)});
  try {
    arm_serial_.Write(frame);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace kros_car

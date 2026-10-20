#include "kros_car/kros_car_system.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

namespace kros_car
{

void KrosCarSystem::wheel_write_line(const std::string & line)
{
  try {
    wheel_serial_.Write(line);
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Wheel serial write error");
  }
}

void KrosCarSystem::wheel_read_available(std::string & line_buffer)
{
  // A feedback line is at most ~175 bytes; anything longer is noise (for
  // example the ESP32 boot log, which is sent at another baud rate).
  constexpr size_t kMaxLineLength = 512;

  try {
    while (wheel_serial_.IsDataAvailable()) {
      const int available = wheel_serial_.GetNumberOfBytesAvailable();
      if (available <= 0) {
        break;
      }
      LibSerial::DataBuffer chunk;
      wheel_serial_.Read(chunk, static_cast<size_t>(available), 5);
      for (const uint8_t byte : chunk) {
        if (byte == '\n') {
          wheel_handle_line(line_buffer);
          line_buffer.clear();
        } else if (line_buffer.size() < kMaxLineLength) {
          line_buffer.push_back(static_cast<char>(byte));
        } else {
          line_buffer.clear();
        }
      }
    }
  } catch (const LibSerial::ReadTimeout &) {
    // The bytes that were announced did not all arrive in time; try again later.
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Wheel serial read error");
  }
}

void KrosCarSystem::wheel_handle_line(const std::string & line)
{
  wheel::Frame frame;
  if (!wheel::parse_frame(line, frame)) {
    if (wheel::is_heartbeat_stop(line)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel controller stopped the wheels: no command within its heartbeat timeout");
    }
    return;
  }

  const auto now_steady = std::chrono::steady_clock::now();
  const rclcpp::Time now_ros = get_clock()->now();
  // Time the frame was sent, on the host clock, without the USB jitter.
  const double sent_s = wheel_clock_mapper_.update(frame.esp_ms, now_ros.seconds());

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sent_s * 1e9), now_ros.get_clock_type());

  {
    std::lock_guard<std::mutex> lock(wheel_fb_mutex_);
    for (size_t i = 0; i < wheel_joint_names_.size(); ++i) {
      const size_t index = static_cast<size_t>(wheel_motor_ids_[i] - 1);
      if (!frame.ok(index)) {
        continue;
      }
      const double direction = wheel_motor_directions_[i];
      WheelFeedback & fb = wheel_feedback_[i];
      fb.position = direction * wheel::counts_to_rad(
        wheel_unwrappers_[i].update(frame.pos_raw[index]));
      fb.velocity = direction * wheel::rpm_to_rad_per_s(frame.spd_rpm[index]);
      fb.effort = direction * wheel::raw_to_ampere(frame.tor_raw[index]) * wheel_torque_constant_;
      fb.mode = frame.mode[index];
      fb.error = frame.err[index];
      fb.stamp = now_steady;
      fb.valid = true;

      msg.name.push_back(wheel_joint_names_[i]);
      msg.position.push_back(fb.position);
      msg.velocity.push_back(fb.velocity);
      msg.effort.push_back(fb.effort);
    }
  }

  if (wheel_feedback_pub_ && !msg.name.empty()) {
    wheel_feedback_pub_->publish(msg);
  }

  for (size_t i = 0; i < wheel_joint_names_.size(); ++i) {
    const int motor_id = wheel_motor_ids_[i];
    const size_t index = static_cast<size_t>(motor_id - 1);
    if (!frame.ok(index)) {
      continue;
    }

    if (frame.err[index] != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel motor %d (%s) reports error 0x%02X: %s",
        motor_id, wheel_joint_names_[i].c_str(), frame.err[index],
        wheel::describe_error(frame.err[index]).c_str());
    }

    // The motors power up in speed loop mode. One that is not was switched by
    // hand; put it back once it is slow enough, at most once a second.
    if (frame.mode[index] != wheel::kModeSpeedLoop &&
      std::abs(frame.spd_rpm[index]) < 10 &&
      now_steady - wheel_mode_fix_sent_[i] > std::chrono::seconds(1))
    {
      RCLCPP_WARN(
        get_logger(),
        "Wheel motor %d (%s) is in mode %d, switching it to speed loop",
        motor_id, wheel_joint_names_[i].c_str(), frame.mode[index]);
      wheel_write_line(wheel::build_change_mode(motor_id, wheel::kModeSpeedLoop));
      wheel_mode_fix_sent_[i] = now_steady;
    }
  }
}

bool KrosCarSystem::wheel_drive_allowed(
  std::chrono::steady_clock::time_point now, std::string & reason)
{
  const auto timeout = std::chrono::milliseconds(wheel_feedback_timeout_ms_);
  std::lock_guard<std::mutex> lock(wheel_fb_mutex_);
  for (size_t i = 0; i < wheel_joint_names_.size(); ++i) {
    const WheelFeedback & fb = wheel_feedback_[i];
    if (!fb.valid || now - fb.stamp > timeout) {
      reason = "no fresh feedback from " + wheel_joint_names_[i];
      return false;
    }
    if (fb.mode != wheel::kModeSpeedLoop) {
      reason = wheel_joint_names_[i] + " is not in speed loop mode";
      return false;
    }
  }
  return true;
}

void KrosCarSystem::wheel_io_loop()
{
  using clock = std::chrono::steady_clock;
  const auto tx_period = std::chrono::duration_cast<clock::duration>(
    std::chrono::duration<double>(1.0 / wheel_command_rate_hz_));
  // Opening the port can reset the ESP32, which then misses whatever is sent
  // while it boots, so the heartbeat setting is repeated.
  const auto heartbeat_period = std::chrono::seconds(2);
  // Feedback only starts after the first command, and the ESP32 may be
  // booting: do not report a missing feedback right away.
  const auto startup_grace = std::chrono::seconds(3);

  wheel_io_start_ = clock::now();
  auto next_tx = wheel_io_start_;
  auto next_heartbeat = wheel_io_start_;
  std::string line_buffer;

  while (wheel_running_.load()) {
    const auto now = clock::now();

    if (now >= next_heartbeat) {
      wheel_write_line(wheel::build_heartbeat(wheel_heartbeat_ms_));
      next_heartbeat = now + heartbeat_period;
    }

    if (now >= next_tx) {
      std::string reason;
      const bool allowed = wheel_drive_allowed(now, reason);
      if (!allowed && !wheel_drive_blocked_ && now - wheel_io_start_ > startup_grace) {
        RCLCPP_ERROR(
          get_logger(), "Wheel feedback problem (%s)%s", reason.c_str(),
          wheel_require_feedback_ ? "; commanding the wheels to stop" : "");
        wheel_drive_blocked_ = true;
      } else if (allowed && wheel_drive_blocked_) {
        RCLCPP_INFO(get_logger(), "Wheel feedback is back");
        wheel_drive_blocked_ = false;
      }

      std::array<int, wheel::kNumWheels> cmd{};
      if (allowed || !wheel_require_feedback_) {
        std::lock_guard<std::mutex> lock(wheel_cmd_mutex_);
        for (const auto & [motor_id, rpm] : latest_wheel_rpms_) {
          cmd[static_cast<size_t>(motor_id - 1)] = static_cast<int>(std::round(rpm));
        }
      }
      wheel_write_line(wheel::build_command_all(cmd, acceleration_act_));

      next_tx += tx_period;
      if (next_tx < now) {
        next_tx = now + tx_period;
      }
    }

    wheel_read_available(line_buffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace kros_car

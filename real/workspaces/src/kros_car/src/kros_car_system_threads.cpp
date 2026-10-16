#include "kros_car/kros_car_system.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
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

void KrosCarSystem::arm_tx_loop()
{
  while (arm_running_.load()) {
    ArmCommand cmd;
    {
      std::lock_guard<std::mutex> lock(arm_cmd_mutex_);
      if (!latest_arm_cmd_.pending) {
        // Nothing new - release lock and sleep
      } else {
        cmd = latest_arm_cmd_;
        latest_arm_cmd_.pending = false;
      }
    }

    if (!cmd.pending) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const auto dequeue_steady = std::chrono::steady_clock::now();
    const auto queue_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      dequeue_steady - cmd.enqueue_steady).count();
    if (queue_ms > 500) {
      RCLCPP_WARN(
        get_logger(),
        "Arm cmd queue delay seq=%llu queue_ms=%lld move_time_ms=%d joints=%zu",
        static_cast<unsigned long long>(cmd.sequence),
        static_cast<long long>(queue_ms),
        cmd.move_time_ms,
        cmd.ticks.size());
    }

    // Mark TX active before serial lock so poll thread yields while TX is waiting/sending.
    arm_tx_in_progress_.store(true, std::memory_order_relaxed);
    const auto lock_wait_start = std::chrono::steady_clock::now();
    size_t sent_count = 0;
    size_t failed_count = 0;
    std::lock_guard<std::mutex> lock(arm_serial_mutex_);
    const auto write_start = std::chrono::steady_clock::now();
    // Send a full joint set in one serial-critical section so poll traffic does not
    // interleave between joints and create visible stagger.
    for (size_t i = 0; i < cmd.ticks.size() && i < arm_servo_ids_.size(); ++i) {
      if (!arm_running_.load()) {
        break;
      }
      const bool ok = arm_move_time_write(
        static_cast<uint8_t>(arm_servo_ids_[i]), cmd.ticks[i], cmd.move_time_ms);
      ++sent_count;
      if (!ok) {
        ++failed_count;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto send_end = std::chrono::steady_clock::now();
    arm_tx_in_progress_.store(false, std::memory_order_relaxed);
    const auto lock_wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      write_start - lock_wait_start).count();
    const auto write_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      send_end - write_start).count();
    const auto send_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      send_end - lock_wait_start).count();
    if (send_ms > 300 || failed_count > 0 || lock_wait_ms > 150) {
      RCLCPP_WARN(
        get_logger(),
        "Arm cmd tx slow/fail seq=%llu queue_ms=%lld lock_wait_ms=%lld write_ms=%lld send_ms=%lld sent=%zu failed=%zu",
        static_cast<unsigned long long>(cmd.sequence),
        static_cast<long long>(queue_ms),
        static_cast<long long>(lock_wait_ms),
        static_cast<long long>(write_ms),
        static_cast<long long>(send_ms),
        sent_count,
        failed_count);
    } else {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Arm cmd tx seq=%llu queue_ms=%lld lock_wait_ms=%lld write_ms=%lld send_ms=%lld sent=%zu",
        static_cast<unsigned long long>(cmd.sequence),
        static_cast<long long>(queue_ms),
        static_cast<long long>(lock_wait_ms),
        static_cast<long long>(write_ms),
        static_cast<long long>(send_ms),
        sent_count);
    }
  }
}

void KrosCarSystem::arm_poll_loop()
{
  int cycle_count = 0;

  while (arm_running_.load()) {
    // Give TX priority when a fresh command is waiting to minimize command latency.
    bool tx_pending = false;
    {
      std::lock_guard<std::mutex> cmd_lock(arm_cmd_mutex_);
      tx_pending = latest_arm_cmd_.pending;
    }
    const bool tx_active = arm_tx_in_progress_.load(std::memory_order_relaxed);
    if (tx_pending || tx_active) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Arm poll yielding for TX command (pending=%d active=%d)",
        tx_pending ? 1 : 0, tx_active ? 1 : 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // Read position for each servo
    for (size_t i = 0; i < arm_servo_ids_.size(); ++i) {
      if (!arm_running_.load()) {
        break;
      }
      if (arm_tx_in_progress_.load(std::memory_order_relaxed)) {
        break;
      }

      std::optional<int> pos;
      {
        std::lock_guard<std::mutex> lock(arm_serial_mutex_);
        pos = arm_read_position(
          static_cast<uint8_t>(arm_servo_ids_[i]), arm_read_timeout_ms_);
      }

      if (pos.has_value()) {
        double rad = ticks_to_rad(pos.value());
        {
          std::lock_guard<std::mutex> lock(arm_state_mutex_);
          latest_arm_positions_[arm_joint_names_[i]] = rad;
          has_arm_feedback_ = true;
        }

        if (arm_joint_names_[i] == gripper_joint_name_) {
          const int gripper_pos_ticks = pos.value();
          const int cmd_ticks = gripper_command_ticks_.load(std::memory_order_relaxed);

          if (gripper_stall_active_.load(std::memory_order_relaxed)) {
            const int hold_ticks = gripper_stall_hold_ticks_.load(std::memory_order_relaxed);
            if (cmd_ticks >= hold_ticks + gripper_stall_release_delta_ticks_) {
              gripper_stall_active_.store(false, std::memory_order_relaxed);
              gripper_stall_tracking_ = false;
              RCLCPP_INFO(
                get_logger(),
                "Gripper stall protection released: cmd_ticks=%d hold_ticks=%d",
                cmd_ticks,
                hold_ticks);
            }
          } else {
            // Detect "over-grip stall": command asks significantly more closing, but position
            // does not continue moving toward that target for a sustained interval.
            if (cmd_ticks + gripper_stall_error_ticks_ < gripper_pos_ticks) {
              const auto now = std::chrono::steady_clock::now();
              if (!gripper_stall_tracking_) {
                gripper_stall_tracking_ = true;
                gripper_stall_start_ = now;
                gripper_stall_reference_ticks_ = gripper_pos_ticks;
              } else {
                const int travel_ticks = std::abs(gripper_pos_ticks - gripper_stall_reference_ticks_);
                if (travel_ticks > gripper_stall_motion_epsilon_ticks_) {
                  // Still moving; refresh observation window.
                  gripper_stall_start_ = now;
                  gripper_stall_reference_ticks_ = gripper_pos_ticks;
                } else {
                  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - gripper_stall_start_).count();
                  if (elapsed_ms >= gripper_stall_trigger_ms_) {
                    const int hold_ticks = std::clamp(
                      gripper_pos_ticks + gripper_stall_backoff_ticks_, 0, gripper_max_ticks_);
                    gripper_stall_hold_ticks_.store(hold_ticks, std::memory_order_relaxed);
                    gripper_stall_active_.store(true, std::memory_order_relaxed);
                    gripper_stall_tracking_ = false;

                    RCLCPP_WARN(
                      get_logger(),
                      "Gripper stall detected: cmd_ticks=%d pos_ticks=%d hold_ticks=%d; blocking further close",
                      cmd_ticks,
                      gripper_pos_ticks,
                      hold_ticks);

                    // Apply immediate relief even if higher layers are holding the same command.
                    std::lock_guard<std::mutex> serial_lock(arm_serial_mutex_);
                    arm_move_time_write(
                      static_cast<uint8_t>(arm_servo_ids_[i]), hold_ticks, default_move_time_ms_);
                  }
                }
              }
            } else {
              gripper_stall_tracking_ = false;
            }
          }
        }
      }
    }

    // Temperature check every N cycles
    ++cycle_count;
    if (cycle_count >= arm_temp_publish_interval_cycles_) {
      cycle_count = 0;

      std::vector<std::optional<int>> arm_temps(arm_servo_ids_.size());
      for (size_t i = 0; i < arm_servo_ids_.size(); ++i) {
        {
          std::lock_guard<std::mutex> lock(arm_serial_mutex_);
          arm_temps[i] = arm_read_temperature(
            static_cast<uint8_t>(arm_servo_ids_[i]), arm_read_timeout_ms_);
        }
      }

      if (arm_temperatures_pub_) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.reserve(arm_temps.size());
        for (const auto & temp_opt : arm_temps) {
          if (temp_opt.has_value()) {
            msg.data.push_back(static_cast<double>(temp_opt.value()));
          } else {
            msg.data.push_back(std::numeric_limits<double>::quiet_NaN());
          }
        }
        arm_temperatures_pub_->publish(msg);
      }

      // Keep existing gripper overheat protection behavior.
      if (!arm_servo_ids_.empty() && !arm_temps.empty() && arm_temps.back().has_value()) {
        int gripper_temp = arm_temps.back().value();

        if (!gripper_overheat_active_.load() && gripper_temp >= gripper_overheat_temp_c_) {
          gripper_overheat_active_.store(true);
          RCLCPP_WARN(get_logger(),
            "Gripper overheat: %dC >= %dC, force opening gripper",
            gripper_temp, gripper_overheat_temp_c_);

          // Force open gripper
          std::lock_guard<std::mutex> lock(arm_serial_mutex_);
          arm_move_time_write(
            static_cast<uint8_t>(arm_servo_ids_.back()), gripper_max_ticks_, 400);
        } else if (gripper_overheat_active_.load()) {
          int recover_temp = gripper_overheat_temp_c_ - gripper_overheat_hysteresis_c_;
          if (gripper_temp <= recover_temp) {
            gripper_overheat_active_.store(false);
            RCLCPP_INFO(get_logger(),
              "Gripper temperature recovered: %dC <= %dC, protection released",
              gripper_temp, recover_temp);
          }
        }
      }
    }
  }
}

}  // namespace kros_car

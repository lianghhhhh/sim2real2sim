#include "kros_car/kros_car_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace kros_car
{

hardware_interface::return_type KrosCarSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Wheel joints: measured by the motors. A wheel without fresh feedback keeps
  // its last position and reports zero velocity and effort; the IO thread is
  // commanding the wheels to stop in that case (see wheel_require_feedback).
  {
    std::lock_guard<std::mutex> lock(wheel_fb_mutex_);
    wheel_feedback_snapshot_ = wheel_feedback_;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(wheel_feedback_timeout_ms_);
  for (size_t i = 0; i < wheel_joint_names_.size(); ++i) {
    const std::string & name = wheel_joint_names_[i];
    const WheelFeedback & fb = wheel_feedback_snapshot_[i];
    const bool fresh = fb.valid && now - fb.stamp <= timeout;
    if (fb.valid) {
      set_state(name + "/position", fb.position);
    }
    set_state(name + "/velocity", fresh ? fb.velocity : 0.0);
    if (wheel_has_effort_state_[i]) {
      set_state(name + "/effort", fresh ? fb.effort : 0.0);
    }
  }

  // Arm joints: use feedback if available, otherwise echo command position
  bool applied_feedback = false;
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    if (has_arm_feedback_) {
      for (const auto & name : arm_joint_names_) {
        auto it = latest_arm_positions_.find(name);
        if (it != latest_arm_positions_.end()) {
          set_state(name + "/position", it->second);
          set_state(name + "/velocity", 0.0);
          applied_feedback = true;
        }
      }
    }
  }

  if (!applied_feedback) {
    for (const auto & name : arm_joint_names_) {
      double cmd_pos = get_command<double>(name + "/position");
      if (std::isnan(cmd_pos)) {
        cmd_pos = 0.0;
      }
      set_state(name + "/position", cmd_pos);
      set_state(name + "/velocity", 0.0);
    }
  }

  // Mirror gripper_joint to gripper_joint_2
  if (!gripper_joint_name_.empty() && !gripper_joint_2_name_.empty()) {
    double pos = get_state<double>(gripper_joint_name_ + "/position");
    double vel = get_state<double>(gripper_joint_name_ + "/velocity");
    set_state(gripper_joint_2_name_ + "/position", -pos);
    set_state(gripper_joint_2_name_ + "/velocity", -vel);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KrosCarSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // Wheel: update latest RPMs for TX thread
  {
    std::lock_guard<std::mutex> lock(wheel_cmd_mutex_);
    for (size_t i = 0; i < wheel_joint_names_.size(); ++i) {
      double vel = get_command<double>(wheel_joint_names_[i] + "/velocity");
      if (std::isnan(vel)) {
        vel = 0.0;
      }
      double rpm = vel * RADS_TO_RPM;
      rpm = std::clamp(rpm, -max_rpm_, max_rpm_);
      double directed_rpm = rpm * wheel_motor_directions_[i];
      latest_wheel_rpms_[wheel_motor_ids_[i]] = directed_rpm;
    }
  }

  // Arm: convert rad commands to ticks, update pending command for TX thread
  std::vector<double> arm_cmd;
  arm_cmd.reserve(arm_joint_names_.size());
  for (const auto & name : arm_joint_names_) {
    double pos = get_command<double>(name + "/position");
    arm_cmd.push_back(std::isnan(pos) ? 0.0 : pos);
  }

  // Only update if command changed, and not faster than the arm bus can take
  // it. A command skipped here still differs from the last one sent, so it is
  // picked up by a later cycle.
  const auto now = std::chrono::steady_clock::now();
  const auto arm_period = std::chrono::duration<double>(1.0 / arm_command_rate_hz_);
  if (arm_cmd != last_arm_command_ && now - last_arm_enqueue_ >= arm_period) {
    last_arm_enqueue_ = now;
    int move_time_ms = std::max({
        default_move_time_ms_,
        static_cast<int>(arm_period.count() * 1000.0),
        static_cast<int>(period.seconds() * 1000.0)});

    std::vector<int> ticks;
    ticks.reserve(arm_joint_names_.size());
    for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
      int t = rad_to_ticks(arm_cmd[i]);
      const bool is_gripper_joint = (arm_joint_names_[i] == gripper_joint_name_);
      if (is_gripper_joint) {
        t = std::min(t, gripper_max_ticks_);
        gripper_command_ticks_.store(t, std::memory_order_relaxed);

        if (gripper_stall_active_.load(std::memory_order_relaxed)) {
          const int hold_ticks = gripper_stall_hold_ticks_.load(std::memory_order_relaxed);
          t = std::max(t, hold_ticks);  // prevent further closing while stalled
        }
      }
      // Override gripper if overheating
      if (is_gripper_joint && gripper_overheat_active_.load()) {
        t = gripper_max_ticks_;  // Force open but keep within safe range
      }
      ticks.push_back(t);
    }

    {
      std::lock_guard<std::mutex> lock(arm_cmd_mutex_);
      latest_arm_cmd_.ticks = std::move(ticks);
      latest_arm_cmd_.move_time_ms = move_time_ms;
      latest_arm_cmd_.sequence = arm_command_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
      latest_arm_cmd_.enqueue_steady = std::chrono::steady_clock::now();
      latest_arm_cmd_.pending = true;

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Arm cmd queued seq=%lu move_time_ms=%d joints=%zu",
        static_cast<unsigned long>(latest_arm_cmd_.sequence),
        latest_arm_cmd_.move_time_ms,
        latest_arm_cmd_.ticks.size());
    }
    last_arm_command_ = arm_cmd;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace kros_car

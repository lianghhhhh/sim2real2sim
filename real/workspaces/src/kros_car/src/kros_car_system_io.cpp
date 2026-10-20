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

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KrosCarSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
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

  return hardware_interface::return_type::OK;
}

}  // namespace kros_car

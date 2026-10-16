#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace kros_car
{

// Drives the wheels from per-wheel speed commands instead of a twist.
//
// It takes the same message the simulated car takes: sensor_msgs/JointState on
// /joint_command, with the wheel speed [rad/s] in `velocity`, looked up by
// joint name. That gives the real car and the simulation one command
// interface, with none of diff_drive_controller's limits or slip correction
// in between.
//
// When no command arrives within `command_timeout` the wheels are commanded
// to stop, so a crashed sender cannot leave the car running.
class WheelVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void on_command(const sensor_msgs::msg::JointState & msg);
  void write_all(const std::vector<double> & velocities);

  std::vector<std::string> joints_;
  std::chrono::duration<double> command_timeout_{0.3};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;

  std::mutex command_mutex_;
  std::vector<double> command_;  // rad/s, same order as joints_
  std::chrono::steady_clock::time_point command_stamp_{};
  bool has_command_{false};

  std::vector<double> output_;   // update() scratch
};

}  // namespace kros_car

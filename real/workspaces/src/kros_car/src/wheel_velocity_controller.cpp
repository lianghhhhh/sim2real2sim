#include "kros_car/wheel_velocity_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace kros_car
{

using CallbackReturn = controller_interface::CallbackReturn;

CallbackReturn WheelVelocityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("command_topic", "/joint_command");
    auto_declare<double>("command_timeout", 0.3);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WheelVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
WheelVelocityController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE, {}};
}

CallbackReturn WheelVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty");
    return CallbackReturn::ERROR;
  }

  const double timeout_s = get_node()->get_parameter("command_timeout").as_double();
  if (!(timeout_s > 0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_timeout' must be positive");
    return CallbackReturn::ERROR;
  }
  command_timeout_ = std::chrono::duration<double>(timeout_s);

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.assign(joints_.size(), 0.0);
    has_command_ = false;
  }
  output_.assign(joints_.size(), 0.0);

  // Best effort, newest only: a late wheel speed command is worse than a
  // dropped one, and this matches both reliable and best-effort publishers.
  const std::string topic = get_node()->get_parameter("command_topic").as_string();
  command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](const sensor_msgs::msg::JointState & msg) {on_command(msg);});

  RCLCPP_INFO(
    get_node()->get_logger(), "Taking wheel speed commands [rad/s] from %s, timeout %.2f s",
    topic.c_str(), timeout_s);
  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Start from a stop: only commands that arrive from now on are used.
  std::lock_guard<std::mutex> lock(command_mutex_);
  has_command_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(output_.begin(), output_.end(), 0.0);
  write_all(output_);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type WheelVelocityController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  bool fresh = false;
  bool stale = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (has_command_) {
      if (std::chrono::steady_clock::now() - command_stamp_ <= command_timeout_) {
        output_ = command_;
        fresh = true;
      } else {
        stale = true;
      }
    }
  }

  if (!fresh) {
    std::fill(output_.begin(), output_.end(), 0.0);
    if (stale) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "No wheel speed command within %.2f s, stopping the wheels", command_timeout_.count());
    }
  }

  write_all(output_);
  return controller_interface::return_type::OK;
}

void WheelVelocityController::write_all(const std::vector<double> & velocities)
{
  for (size_t i = 0; i < command_interfaces_.size() && i < velocities.size(); ++i) {
    if (!command_interfaces_[i].set_value(velocities[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Could not write the velocity command of %s", joints_[i].c_str());
    }
  }
}

void WheelVelocityController::on_command(const sensor_msgs::msg::JointState & msg)
{
  auto & clock = *get_node()->get_clock();
  const auto logger = get_node()->get_logger();

  if (msg.velocity.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, 2000,
      "Ignoring a command without velocity: this controller takes wheel speeds [rad/s], "
      "not effort or position");
    return;
  }

  std::vector<double> command(joints_.size(), 0.0);
  if (msg.name.empty()) {
    if (msg.velocity.size() != joints_.size()) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, 2000,
        "Ignoring a command without joint names that has %zu velocities, expected %zu",
        msg.velocity.size(), joints_.size());
      return;
    }
    command = msg.velocity;
  } else {
    if (msg.name.size() != msg.velocity.size()) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, 2000, "Ignoring a command whose name and velocity sizes differ");
      return;
    }
    // A wheel the message does not name is commanded to stop.
    bool matched = false;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), joints_[i]);
      if (it != msg.name.end()) {
        command[i] = msg.velocity[static_cast<size_t>(it - msg.name.begin())];
        matched = true;
      }
    }
    if (!matched) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, 2000, "Ignoring a command that names none of this controller's joints");
      return;
    }
  }

  if (!std::all_of(command.begin(), command.end(), [](double v) {return std::isfinite(v);})) {
    RCLCPP_WARN_THROTTLE(logger, clock, 2000, "Ignoring a command with a non-finite velocity");
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  command_ = command;
  command_stamp_ = std::chrono::steady_clock::now();
  has_command_ = true;
}

}  // namespace kros_car

PLUGINLIB_EXPORT_CLASS(kros_car::WheelVelocityController, controller_interface::ControllerInterface)

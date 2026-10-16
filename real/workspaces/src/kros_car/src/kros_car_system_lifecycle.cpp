#include "kros_car/kros_car_system.hpp"

#include <algorithm>
#include <chrono>
#include <set>

#include "pluginlib/class_list_macros.hpp"

namespace kros_car
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

hardware_interface::CallbackReturn KrosCarSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto get_param = [this](const std::string & key) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    if (it != info_.hardware_parameters.end()) {
      return it->second;
    }
    return "";
  };

  // Wheel config
  wheel_port_ = get_param("wheel_port");
  if (wheel_port_.empty()) {
    wheel_port_ = "/dev/usb_wheel";
  }
  wheel_baudrate_ = parse_int_or_default(get_param("wheel_baudrate"), 460800);
  max_rpm_ = parse_double_or_default(get_param("max_rpm"), 100.0);
  acceleration_act_ = parse_int_or_default(get_param("acceleration_act"), 1);
  wheel_radius_ = parse_double_or_default(get_param("wheel_radius"), 0.05035);
  // The firmware polls the motors at 50 Hz, so a faster command rate gains nothing.
  wheel_command_rate_hz_ = std::clamp(
    parse_double_or_default(get_param("wheel_command_rate_hz"), 50.0), 1.0, 100.0);
  wheel_heartbeat_ms_ = parse_int_or_default(get_param("wheel_heartbeat_ms"), 500);
  if (wheel_heartbeat_ms_ <= 0) {
    wheel_heartbeat_ms_ = -1;  // firmware convention for "off"
  }
  wheel_feedback_timeout_ms_ = std::max(
    1, parse_int_or_default(get_param("wheel_feedback_timeout_ms"), 200));
  wheel_torque_constant_ = parse_double_or_default(get_param("wheel_torque_constant"), 0.75);
  wheel_require_feedback_ = parse_bool_or_default(get_param("wheel_require_feedback"), true);

  auto motor_ids_str = get_param("wheel_motor_ids");
  if (!motor_ids_str.empty()) {
    for (const auto & s : split_csv(motor_ids_str)) {
      wheel_motor_ids_.push_back(parse_int_or_default(s, 0));
    }
  }
  if (wheel_motor_ids_.empty()) {
    wheel_motor_ids_ = {1, 2, 3, 4};
  }

  auto motor_dirs_str = get_param("wheel_motor_directions");
  if (!motor_dirs_str.empty()) {
    for (const auto & s : split_csv(motor_dirs_str)) {
      wheel_motor_directions_.push_back(parse_double_or_default(s, 1.0));
    }
  }
  if (wheel_motor_directions_.empty()) {
    wheel_motor_directions_ = {1.0, 1.0, -1.0, -1.0};
  }

  // Arm config
  arm_port_ = get_param("arm_port");
  if (arm_port_.empty()) {
    arm_port_ = "/dev/usb_robot_arm";
  }
  arm_baudrate_ = parse_int_or_default(get_param("arm_baudrate"), 115200);
  default_move_time_ms_ = parse_int_or_default(get_param("default_move_time_ms"), 80);
  arm_read_timeout_ms_ = parse_int_or_default(get_param("arm_read_timeout_ms"), 80);
  arm_temp_publish_interval_cycles_ = std::max(
    1, parse_int_or_default(get_param("arm_temp_publish_interval_cycles"), 10));
  arm_command_rate_hz_ = std::clamp(
    parse_double_or_default(get_param("arm_command_rate_hz"), 20.0), 1.0, 100.0);
  gripper_max_ticks_ = std::clamp(parse_int_or_default(get_param("gripper_max_ticks"), 1000), 0, 1000);
  gripper_stall_error_ticks_ = std::max(0, parse_int_or_default(get_param("gripper_stall_error_ticks"), 30));
  gripper_stall_trigger_ms_ = std::max(0, parse_int_or_default(get_param("gripper_stall_trigger_ms"), 500));
  gripper_stall_motion_epsilon_ticks_ = std::max(
    0, parse_int_or_default(get_param("gripper_stall_motion_epsilon_ticks"), 3));
  gripper_stall_backoff_ticks_ = std::max(0, parse_int_or_default(get_param("gripper_stall_backoff_ticks"), 10));
  gripper_stall_release_delta_ticks_ = std::max(
    0, parse_int_or_default(get_param("gripper_stall_release_delta_ticks"), 20));
  gripper_overheat_temp_c_ = parse_int_or_default(get_param("gripper_overheat_temp_c"), 70);
  gripper_overheat_hysteresis_c_ = parse_int_or_default(
    get_param("gripper_overheat_hysteresis_c"), 5);

  auto servo_ids_str = get_param("arm_servo_ids");
  if (!servo_ids_str.empty()) {
    for (const auto & s : split_csv(servo_ids_str)) {
      arm_servo_ids_.push_back(parse_int_or_default(s, 0));
    }
  }
  if (arm_servo_ids_.empty()) {
    arm_servo_ids_ = {1, 2, 3};
  }

  // Joint names from URDF params
  auto wheel_joints = split_csv(
    "front_left_joint,rear_left_joint,front_right_joint,rear_right_joint");
  auto arm_joints = split_csv("arm_1_joint,arm_2_joint,gripper_joint");

  auto wj_str = get_param("wheel_joints");
  if (!wj_str.empty()) {
    wheel_joints = split_csv(wj_str);
  }
  auto aj_str = get_param("arm_joints");
  if (!aj_str.empty()) {
    arm_joints = split_csv(aj_str);
  }

  // Cross-reference with URDF joints
  std::set<std::string> urdf_joints;
  for (const auto & joint : info_.joints) {
    urdf_joints.insert(joint.name);
  }

  wheel_joint_names_.clear();
  for (const auto & name : wheel_joints) {
    if (urdf_joints.count(name)) {
      wheel_joint_names_.push_back(name);
    }
  }

  arm_joint_names_.clear();
  for (const auto & name : arm_joints) {
    if (urdf_joints.count(name)) {
      arm_joint_names_.push_back(name);
    }
  }

  gripper_joint_name_.clear();
  gripper_joint_2_name_.clear();
  if (urdf_joints.count("gripper_joint")) {
    gripper_joint_name_ = "gripper_joint";
  }
  if (urdf_joints.count("gripper_joint_2")) {
    gripper_joint_2_name_ = "gripper_joint_2";
  }

  if (wheel_joint_names_.empty() || arm_joint_names_.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to resolve wheel/arm joints from URDF");
    return CallbackReturn::ERROR;
  }

  // Validate motor count matches joint count
  if (wheel_motor_ids_.size() != wheel_joint_names_.size()) {
    RCLCPP_ERROR(get_logger(),
      "wheel_motor_ids size (%zu) != wheel_joints size (%zu)",
      wheel_motor_ids_.size(), wheel_joint_names_.size());
    return CallbackReturn::ERROR;
  }
  if (wheel_motor_directions_.size() != wheel_joint_names_.size()) {
    RCLCPP_ERROR(get_logger(),
      "wheel_motor_directions size (%zu) != wheel_joints size (%zu)",
      wheel_motor_directions_.size(), wheel_joint_names_.size());
    return CallbackReturn::ERROR;
  }
  if (arm_servo_ids_.size() != arm_joint_names_.size()) {
    RCLCPP_ERROR(get_logger(),
      "arm_servo_ids size (%zu) != arm_joints size (%zu)",
      arm_servo_ids_.size(), arm_joint_names_.size());
    return CallbackReturn::ERROR;
  }

  // The wheel controller firmware polls motor IDs 1..4 and reports them by position.
  std::set<int> unique_motor_ids(wheel_motor_ids_.begin(), wheel_motor_ids_.end());
  if (unique_motor_ids.size() != wheel_motor_ids_.size() ||
    *unique_motor_ids.begin() < 1 ||
    *unique_motor_ids.rbegin() > static_cast<int>(wheel::kNumWheels))
  {
    RCLCPP_ERROR(get_logger(),
      "wheel_motor_ids must be unique and within 1..%zu", wheel::kNumWheels);
    return CallbackReturn::ERROR;
  }
  if (wheel_heartbeat_ms_ > 0 && wheel_heartbeat_ms_ < 3000.0 / wheel_command_rate_hz_) {
    RCLCPP_WARN(get_logger(),
      "wheel_heartbeat_ms=%d is shorter than three command periods at %.1f Hz; "
      "the wheels may stop on ordinary jitter",
      wheel_heartbeat_ms_, wheel_command_rate_hz_);
  }

  // Initialize wheel RPM map
  for (int mid : wheel_motor_ids_) {
    latest_wheel_rpms_[mid] = 0.0;
  }

  wheel_feedback_.assign(wheel_joint_names_.size(), WheelFeedback{});
  wheel_feedback_snapshot_ = wheel_feedback_;
  wheel_has_effort_state_.assign(wheel_joint_names_.size(), false);
  wheel_unwrappers_.assign(wheel_joint_names_.size(), wheel::PositionUnwrapper{});
  wheel_mode_fix_sent_.assign(
    wheel_joint_names_.size(), std::chrono::steady_clock::time_point{});

  last_arm_command_.clear();

  RCLCPP_INFO(get_logger(), "KrosCarSystem initialized: wheel_port=%s, arm_port=%s",
    wheel_port_.c_str(), arm_port_.c_str());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KrosCarSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify required interfaces exist
  for (const auto & name : wheel_joint_names_) {
    if (!has_command(name + "/velocity") ||
      !has_state(name + "/position") ||
      !has_state(name + "/velocity"))
    {
      RCLCPP_ERROR(get_logger(), "Missing interfaces for wheel joint: %s", name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & name : arm_joint_names_) {
    if (!has_command(name + "/position") ||
      !has_state(name + "/position") ||
      !has_state(name + "/velocity"))
    {
      RCLCPP_ERROR(get_logger(), "Missing interfaces for arm joint: %s", name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Initialize all state interfaces to zero
  for (const auto & joint : info_.joints) {
    const std::string pos_key = joint.name + "/position";
    const std::string vel_key = joint.name + "/velocity";
    if (has_state(pos_key)) {
      set_state(pos_key, 0.0);
    }
    if (has_state(vel_key)) {
      set_state(vel_key, 0.0);
    }
  }

  // The effort state is optional so an older URDF without it still works.
  for (size_t i = 0; i < wheel_joint_names_.size(); ++i) {
    const std::string effort_key = wheel_joint_names_[i] + "/effort";
    wheel_has_effort_state_[i] = has_state(effort_key);
    if (wheel_has_effort_state_[i]) {
      set_state(effort_key, 0.0);
    } else {
      RCLCPP_WARN(get_logger(),
        "Wheel joint %s has no effort state interface; motor current will not be in /joint_states",
        wheel_joint_names_[i].c_str());
    }
  }

  // Wheel positions restart from zero on every activation.
  {
    std::lock_guard<std::mutex> lock(wheel_fb_mutex_);
    wheel_feedback_.assign(wheel_joint_names_.size(), WheelFeedback{});
  }
  wheel_unwrappers_.assign(wheel_joint_names_.size(), wheel::PositionUnwrapper{});
  wheel_mode_fix_sent_.assign(
    wheel_joint_names_.size(), std::chrono::steady_clock::time_point{});
  wheel_clock_mapper_.reset();
  wheel_drive_blocked_ = false;
  {
    std::lock_guard<std::mutex> lock(wheel_cmd_mutex_);
    for (auto & [motor_id, rpm] : latest_wheel_rpms_) {
      (void)motor_id;
      rpm = 0.0;
    }
  }

  last_arm_command_.clear();
  has_arm_feedback_ = false;
  gripper_overheat_active_.store(false);
  gripper_command_ticks_.store(gripper_max_ticks_);
  gripper_stall_active_.store(false);
  gripper_stall_hold_ticks_.store(gripper_max_ticks_);
  gripper_stall_tracking_ = false;
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    latest_arm_positions_.clear();
  }
  arm_temperatures_pub_.reset();
  wheel_feedback_pub_.reset();
  last_arm_enqueue_ = std::chrono::steady_clock::time_point{};

  // Open wheel serial port
  try {
    wheel_serial_.Open(wheel_port_);
    if (!set_baud_rate(wheel_serial_, wheel_baudrate_)) {
      RCLCPP_ERROR(get_logger(), "Unsupported wheel_baudrate: %d", wheel_baudrate_);
      try { wheel_serial_.Close(); } catch (...) {}
      return CallbackReturn::ERROR;
    }
    wheel_serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    wheel_serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    wheel_serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    wheel_serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    wheel_serial_.SetDTR(false);
    wheel_serial_.SetRTS(false);
    wheel_serial_.FlushIOBuffers();
    RCLCPP_INFO(get_logger(), "Wheel serial opened: %s @ %d",
      wheel_port_.c_str(), wheel_baudrate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open wheel serial %s: %s",
      wheel_port_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  // Open arm serial port
  try {
    arm_serial_.Open(arm_port_);
    if (!set_baud_rate(arm_serial_, arm_baudrate_)) {
      RCLCPP_ERROR(get_logger(), "Unsupported arm_baudrate: %d", arm_baudrate_);
      try { arm_serial_.Close(); } catch (...) {}
      try { wheel_serial_.Close(); } catch (...) {}
      return CallbackReturn::ERROR;
    }
    arm_serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    arm_serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    arm_serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    arm_serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    arm_serial_.FlushIOBuffers();
    RCLCPP_INFO(get_logger(), "Arm serial opened: %s @ %d",
      arm_port_.c_str(), arm_baudrate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open arm serial %s: %s",
      arm_port_.c_str(), e.what());
    // Close wheel serial since we're failing
    try { wheel_serial_.Close(); } catch (...) {}
    return CallbackReturn::ERROR;
  }

  // Enable torque on arm servos
  for (int sid : arm_servo_ids_) {
    std::lock_guard<std::mutex> lock(arm_serial_mutex_);
    arm_set_torque(static_cast<uint8_t>(sid), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Hold current arm pose on startup to avoid jumping to default zeros.
  bool have_full_startup_pose = true;
  std::vector<double> startup_arm_command;
  startup_arm_command.reserve(arm_joint_names_.size());
  for (size_t i = 0; i < arm_servo_ids_.size() && i < arm_joint_names_.size(); ++i) {
    std::optional<int> pos;
    {
      std::lock_guard<std::mutex> lock(arm_serial_mutex_);
      pos = arm_read_position(static_cast<uint8_t>(arm_servo_ids_[i]), arm_read_timeout_ms_);
    }

    if (!pos.has_value()) {
      have_full_startup_pose = false;
      break;
    }

    const double rad = ticks_to_rad(pos.value());
    const std::string & joint = arm_joint_names_[i];
    startup_arm_command.push_back(rad);
    if (joint == gripper_joint_name_) {
      gripper_command_ticks_.store(std::clamp(pos.value(), 0, gripper_max_ticks_));
      gripper_stall_hold_ticks_.store(gripper_command_ticks_.load(std::memory_order_relaxed));
    }

    set_state(joint + "/position", rad);
    set_state(joint + "/velocity", 0.0);
    set_command(joint + "/position", rad);
  }

  if (have_full_startup_pose && startup_arm_command.size() == arm_joint_names_.size()) {
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      has_arm_feedback_ = true;
      for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
        latest_arm_positions_[arm_joint_names_[i]] = startup_arm_command[i];
      }
    }
    last_arm_command_ = startup_arm_command;
    RCLCPP_INFO(get_logger(), "Initialized arm startup pose from live servo positions");
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Failed to read full startup arm pose; startup command may fall back to zero defaults");
  }

  if (auto node = get_node()) {
    arm_temperatures_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_joint_temperatures", rclcpp::SystemDefaultsQoS());
    wheel_feedback_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "/wheel_feedback", rclcpp::SensorDataQoS());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Hardware node not available; /arm_joint_temperatures and /wheel_feedback "
      "will not be published");
  }

  // Start threads
  wheel_running_.store(true);
  wheel_io_thread_ = std::thread(&KrosCarSystem::wheel_io_loop, this);

  arm_running_.store(true);
  arm_tx_thread_ = std::thread(&KrosCarSystem::arm_tx_loop, this);
  arm_poll_thread_ = std::thread(&KrosCarSystem::arm_poll_loop, this);

  RCLCPP_INFO(get_logger(), "KrosCarSystem activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KrosCarSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop threads
  wheel_running_.store(false);
  arm_running_.store(false);

  if (wheel_io_thread_.joinable()) {
    wheel_io_thread_.join();
  }
  if (arm_tx_thread_.joinable()) {
    arm_tx_thread_.join();
  }
  if (arm_poll_thread_.joinable()) {
    arm_poll_thread_.join();
  }

  // Send zero to all wheel motors. The IO thread has stopped, so nothing else
  // is writing to the port; the firmware heartbeat stops them anyway if this
  // line is lost.
  wheel_write_line(wheel::build_command_all({}, acceleration_act_));

  // Stop all arm servos
  for (int sid : arm_servo_ids_) {
    try {
      std::lock_guard<std::mutex> lock(arm_serial_mutex_);
      arm_move_stop(static_cast<uint8_t>(sid));
    } catch (...) {}
  }

  // Close serial ports
  try { wheel_serial_.Close(); } catch (...) {}
  try { arm_serial_.Close(); } catch (...) {}
  arm_temperatures_pub_.reset();
  wheel_feedback_pub_.reset();

  RCLCPP_INFO(get_logger(), "KrosCarSystem deactivated");
  return CallbackReturn::SUCCESS;
}

KrosCarSystem::~KrosCarSystem()
{
  wheel_running_.store(false);
  arm_running_.store(false);

  if (wheel_io_thread_.joinable()) {
    wheel_io_thread_.join();
  }
  if (arm_tx_thread_.joinable()) {
    arm_tx_thread_.join();
  }
  if (arm_poll_thread_.joinable()) {
    arm_poll_thread_.join();
  }

  try { wheel_serial_.Close(); } catch (...) {}
  try { arm_serial_.Close(); } catch (...) {}
}

}  // namespace kros_car

PLUGINLIB_EXPORT_CLASS(kros_car::KrosCarSystem, hardware_interface::SystemInterface)

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <libserial/SerialPort.h>

#include "kros_car/wheel_protocol.hpp"

namespace kros_car
{

class KrosCarSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  ~KrosCarSystem() override;

private:
  // Utility
  static std::vector<std::string> split_csv(const std::string & raw);
  static int parse_int_or_default(const std::string & value, int fallback);
  static double parse_double_or_default(const std::string & value, double fallback);
  static bool parse_bool_or_default(const std::string & value, bool fallback);
  static bool set_baud_rate(LibSerial::SerialPort & port, int baudrate);

  // Wheel helpers. Only the wheel IO thread touches the wheel serial port
  // while it runs.
  void wheel_write_line(const std::string & line);
  void wheel_read_available(std::string & line_buffer);
  void wheel_handle_line(const std::string & line);
  bool wheel_drive_allowed(std::chrono::steady_clock::time_point now, std::string & reason);

  // Thread loops
  void wheel_io_loop();

  // Conversion helpers
  static constexpr double RADS_TO_RPM = 60.0 / (2.0 * 3.14159265358979323846);

  // Joint names
  std::vector<std::string> wheel_joint_names_;

  // Wheel serial and config
  LibSerial::SerialPort wheel_serial_;
  std::string wheel_port_;
  int wheel_baudrate_{460800};
  std::vector<int> wheel_motor_ids_;
  std::vector<double> wheel_motor_directions_;
  double max_rpm_{100.0};
  int acceleration_act_{1};
  double wheel_radius_{0.05035};
  double wheel_command_rate_hz_{50.0};
  int wheel_heartbeat_ms_{500};
  int wheel_feedback_timeout_ms_{200};
  // DDSM115 datasheet value; measure it on the car before trusting efforts.
  double wheel_torque_constant_{0.75};
  // When true the wheels are commanded to stop while any wheel has no fresh
  // feedback or is not in speed loop mode.
  bool wheel_require_feedback_{true};

  // Wheel IO thread: sends the targets at a fixed rate and reads feedback.
  std::thread wheel_io_thread_;
  std::mutex wheel_cmd_mutex_;
  std::atomic<bool> wheel_running_{false};
  std::map<int, double> latest_wheel_rpms_;  // motor_id -> signed RPM

  // Wheel feedback, written by the IO thread and read by read().
  struct WheelFeedback
  {
    bool valid{false};      // at least one reading received since activation
    double position{0.0};   // rad, joint sign, relative to the first reading
    double velocity{0.0};   // rad/s, joint sign
    double effort{0.0};     // N*m, joint sign (torque current * torque constant)
    int mode{0};
    int error{0};
    std::chrono::steady_clock::time_point stamp{};
  };
  std::mutex wheel_fb_mutex_;
  std::vector<WheelFeedback> wheel_feedback_;  // same order as wheel_joint_names_
  std::vector<WheelFeedback> wheel_feedback_snapshot_;  // read() scratch copy
  std::vector<bool> wheel_has_effort_state_;

  // IO-thread-only wheel state
  std::vector<wheel::PositionUnwrapper> wheel_unwrappers_;
  wheel::ClockMapper wheel_clock_mapper_;
  std::vector<std::chrono::steady_clock::time_point> wheel_mode_fix_sent_;
  std::chrono::steady_clock::time_point wheel_io_start_{};
  bool wheel_drive_blocked_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_feedback_pub_;
};

}  // namespace kros_car

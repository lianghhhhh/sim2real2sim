#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <libserial/SerialPort.h>

#include "kros_car/wheel_protocol.hpp"

namespace kros_car
{

struct ArmServoPacket
{
  uint8_t servo_id;
  uint8_t length;
  uint8_t cmd;
  std::vector<uint8_t> params;
};

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

  // Arm servo protocol helpers
  std::vector<uint8_t> arm_build_packet(
    uint8_t servo_id, uint8_t cmd, const std::vector<uint8_t> & params);
  std::optional<ArmServoPacket> arm_read_packet(int timeout_ms);
  bool arm_move_time_write(uint8_t servo_id, int position_ticks, int time_ms);
  bool arm_move_stop(uint8_t servo_id);
  std::optional<int> arm_read_position(uint8_t servo_id, int timeout_ms);
  std::optional<int> arm_read_temperature(uint8_t servo_id, int timeout_ms);
  bool arm_set_torque(uint8_t servo_id, bool enable);

  // Wheel helpers. Only the wheel IO thread touches the wheel serial port
  // while it runs.
  void wheel_write_line(const std::string & line);
  void wheel_read_available(std::string & line_buffer);
  void wheel_handle_line(const std::string & line);
  bool wheel_drive_allowed(std::chrono::steady_clock::time_point now, std::string & reason);

  // Thread loops
  void wheel_io_loop();
  void arm_tx_loop();
  void arm_poll_loop();

  // Conversion helpers
  static constexpr double RADS_TO_RPM = 60.0 / (2.0 * 3.14159265358979323846);
  static double ticks_to_rad(int ticks);
  static int rad_to_ticks(double rad);

  // Joint names
  std::vector<std::string> wheel_joint_names_;
  std::vector<std::string> arm_joint_names_;
  std::string gripper_joint_name_;
  std::string gripper_joint_2_name_;

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

  // Arm serial and config
  LibSerial::SerialPort arm_serial_;
  std::string arm_port_;
  int arm_baudrate_{115200};
  std::vector<int> arm_servo_ids_;
  int default_move_time_ms_{80};
  int arm_read_timeout_ms_{80};
  int arm_temp_publish_interval_cycles_{10};
  // The arm bus cannot take commands at the controller rate without starving
  // the position poll, so arm targets are forwarded at this rate at most.
  double arm_command_rate_hz_{20.0};
  int gripper_max_ticks_{1000};
  int gripper_stall_error_ticks_{30};
  int gripper_stall_trigger_ms_{500};
  int gripper_stall_motion_epsilon_ticks_{3};
  int gripper_stall_backoff_ticks_{10};
  int gripper_stall_release_delta_ticks_{20};
  int gripper_overheat_temp_c_{70};
  int gripper_overheat_hysteresis_c_{5};

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

  // Arm TX thread
  std::thread arm_tx_thread_;
  std::mutex arm_cmd_mutex_;
  struct ArmCommand {
    std::vector<int> ticks;   // per-servo tick targets
    int move_time_ms{80};
    bool pending{false};      // true when write() has new data
    uint64_t sequence{0};     // monotonically increasing command id
    std::chrono::steady_clock::time_point enqueue_steady{};  // queue timestamp
  };
  ArmCommand latest_arm_cmd_;
  std::atomic<uint64_t> arm_command_sequence_{0};

  // Arm poll thread
  std::thread arm_poll_thread_;
  std::mutex arm_serial_mutex_;
  std::atomic<bool> arm_running_{false};
  std::atomic<bool> arm_tx_in_progress_{false};

  // Arm state feedback
  std::mutex arm_state_mutex_;
  std::map<std::string, double> latest_arm_positions_;  // joint_name -> radians
  bool has_arm_feedback_{false};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_temperatures_pub_;

  // Arm command change detection (used by write() to avoid redundant updates)
  std::vector<double> last_arm_command_;
  std::chrono::steady_clock::time_point last_arm_enqueue_{};

  // Gripper overheat state shared between poll and write threads
  std::atomic<bool> gripper_overheat_active_{false};
  std::atomic<int> gripper_command_ticks_{300};
  std::atomic<bool> gripper_stall_active_{false};
  std::atomic<int> gripper_stall_hold_ticks_{300};

  // Poll-thread-only gripper stall tracking state
  bool gripper_stall_tracking_{false};
  int gripper_stall_reference_ticks_{0};
  std::chrono::steady_clock::time_point gripper_stall_start_{};  // valid when tracking=true
};

}  // namespace kros_car

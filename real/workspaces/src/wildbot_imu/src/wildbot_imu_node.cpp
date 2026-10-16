#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <thread>
#include <unistd.h>

#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{
constexpr uint8_t kFrameHeader = 0x55;
constexpr uint8_t kTimeFrame = 0x50;
constexpr uint8_t kAccelerationFrame = 0x51;
constexpr uint8_t kGyroFrame = 0x52;
constexpr uint8_t kAngleFrame = 0x53;
constexpr uint8_t kMagFrame = 0x54;
constexpr uint8_t kGpsAccuracyFrame = 0x5A;
constexpr uint8_t kReadRegFrame = 0x5F;
constexpr std::size_t kFrameLength = 11;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

// Write protocol: 0xFF 0xAA <reg> <DATAL> <DATAH>.
constexpr uint8_t kWriteHeader0 = 0xFF;
constexpr uint8_t kWriteHeader1 = 0xAA;
constexpr uint8_t kRegCalsw = 0x01;
constexpr uint8_t kRegAxis6 = 0x24;
constexpr uint8_t kRegKey = 0x69;
constexpr uint16_t kKeyUnlock = 0xB588;
constexpr uint16_t kCalswSetReference = 0x0008;  // capture current pose as zero
constexpr uint16_t kAxis6On = 0x0001;  // 6-axis algorithm, no magnetometer fusion

// Covariances derived from Yahboom 10-axis IMU Navigation Module spec (worst-case RMS noise @100 Hz BW).
// Accel noise 1 mg-rms -> (0.001 * g)^2
constexpr double kLinearAccelVariance = (0.001 * kGravity) * (0.001 * kGravity);
// Gyro noise 0.07 deg/s-rms -> (0.07 * pi/180)^2
constexpr double kAngularVelocityVariance =
  (0.07 * kPi / 180.0) * (0.07 * kPi / 180.0);
// Dynamic inclination accuracy 0.5 deg (roll, pitch)
constexpr double kOrientationRollPitchVariance =
  (0.5 * kPi / 180.0) * (0.5 * kPi / 180.0);
// 6-axis mode: yaw is integrated from gyro and drifts. Datasheet 1-deg figure
// applies to 9-axis only; pick a value that reflects realistic accumulated error.
constexpr double kOrientationYawVariance =
  (5.0 * kPi / 180.0) * (5.0 * kPi / 180.0);
}  // namespace

class WildbotImuNode : public rclcpp::Node
{
public:
  WildbotImuNode()
  : Node("imu_driver_node"),
    running_(true)
  {
    port_name_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    disable_magnetometer_ = declare_parameter<bool>("disable_magnetometer", false);
    zero_reference_at_startup_ = declare_parameter<bool>("zero_reference_at_startup", true);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    read_thread_ = std::thread(&WildbotImuNode::readLoop, this);
  }

  ~WildbotImuNode() override
  {
    running_.store(false);

    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }

    if (read_thread_.joinable()) {
      read_thread_.join();
    }
  }

private:
  struct ImuState
  {
    std::array<double, 3> acceleration{{0.0, 0.0, 0.0}};
    std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};
    std::array<double, 3> angle_degree{{0.0, 0.0, 0.0}};
    std::array<double, 3> magnetometer{{0.0, 0.0, 0.0}};
  };

  static speed_t baudToTermios(int baud_rate)
  {
    switch (baud_rate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      default:
        return 0;
    }
  }

  static int16_t parseInt16(const uint8_t low, const uint8_t high)
  {
    return static_cast<int16_t>(
      static_cast<uint16_t>(low) |
      (static_cast<uint16_t>(high) << 8));
  }

  bool openSerialPort()
  {
    const speed_t speed = baudToTermios(baud_rate_);
    if (speed == 0) {
      RCLCPP_ERROR(get_logger(), "Unsupported baud rate: %d", baud_rate_);
      return false;
    }

    const int fd = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to open %s: %s", port_name_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr failed on %s: %s", port_name_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr failed on %s: %s", port_name_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    serial_fd_ = fd;
    RCLCPP_INFO(get_logger(), "Serial port opened: %s @ %d", port_name_.c_str(), baud_rate_);
    return true;
  }

  void closeSerialPort()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  bool writeRegister(const uint8_t reg, const uint16_t value)
  {
    if (serial_fd_ < 0) {
      return false;
    }
    const uint8_t cmd[5] = {
      kWriteHeader0,
      kWriteHeader1,
      reg,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
    };
    const ssize_t written = ::write(serial_fd_, cmd, sizeof(cmd));
    return written == static_cast<ssize_t>(sizeof(cmd));
  }

  // Unlock + apply per-session config. Not saved to flash, so this re-runs on every start.
  void configureDevice()
  {
    if (!writeRegister(kRegKey, kKeyUnlock)) {
      RCLCPP_WARN(get_logger(), "Failed to send unlock command");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (disable_magnetometer_) {
      if (writeRegister(kRegAxis6, kAxis6On)) {
        RCLCPP_INFO(get_logger(), "Magnetometer fusion disabled (6-axis mode)");
      } else {
        RCLCPP_WARN(get_logger(), "Failed to set AXIS6 register");
      }
      // Let the heading filter settle on the new mode before zeroing.
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    if (zero_reference_at_startup_) {
      if (writeRegister(kRegCalsw, kCalswSetReference)) {
        RCLCPP_INFO(get_logger(), "Current orientation captured as zero reference");
      } else {
        RCLCPP_WARN(get_logger(), "Failed to set angle reference");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void readLoop()
  {
    std::array<uint8_t, 256> buffer {};

    while (running_.load() && rclcpp::ok()) {
      if (serial_fd_ < 0) {
        if (!openSerialPort()) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        configureDevice();
      }

      const ssize_t bytes_read = ::read(serial_fd_, buffer.data(), buffer.size());
      if (bytes_read > 0) {
        for (ssize_t i = 0; i < bytes_read; ++i) {
          handleByte(buffer[static_cast<std::size_t>(i)]);
        }
        continue;
      }

      if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(get_logger(), "Serial read failed: %s", std::strerror(errno));
        closeSerialPort();
      }
    }
  }

  void resetFrame()
  {
    frame_index_ = 0;
  }

  bool checksumValid() const
  {
    uint32_t sum = 0;
    for (std::size_t i = 0; i < kFrameLength - 1; ++i) {
      sum += frame_[i];
    }
    return static_cast<uint8_t>(sum & 0xFF) == frame_[kFrameLength - 1];
  }

  void handleByte(const uint8_t byte)
  {
    if (frame_index_ == 0U) {
      if (byte != kFrameHeader) {
        return;
      }
      frame_start_time_ = now();
    } else if (frame_index_ == 1U) {
      const bool valid_type =
        (byte >= kTimeFrame && byte <= kGpsAccuracyFrame) || byte == kReadRegFrame;
      if (!valid_type) {
        resetFrame();
        return;
      }
    }

    frame_[frame_index_++] = byte;

    if (frame_index_ < kFrameLength) {
      return;
    }

    if (checksumValid()) {
      decodeFrame();
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Dropped IMU frame with invalid checksum");
    }

    resetFrame();
  }

  void decodeFrame()
  {
    const auto x_raw = parseInt16(frame_[2], frame_[3]);
    const auto y_raw = parseInt16(frame_[4], frame_[5]);
    const auto z_raw = parseInt16(frame_[6], frame_[7]);

    switch (frame_[1]) {
      case kAccelerationFrame:
      {
        state_.acceleration = {
          x_raw / 32768.0 * 16.0 * kGravity,
          y_raw / 32768.0 * 16.0 * kGravity,
          z_raw / 32768.0 * 16.0 * kGravity
        };
        break;
      }
      case kGyroFrame:
      {
        state_.angular_velocity = {
          x_raw / 32768.0 * 2000.0 * kPi / 180.0,
          y_raw / 32768.0 * 2000.0 * kPi / 180.0,
          z_raw / 32768.0 * 2000.0 * kPi / 180.0
        };
        break;
      }
      case kAngleFrame:
      {
        state_.angle_degree = {
          x_raw / 32768.0 * 180.0,
          y_raw / 32768.0 * 180.0,
          z_raw / 32768.0 * 180.0
        };
        publishImu();
        break;
      }
      case kMagFrame:
      {
        state_.magnetometer = {
          static_cast<double>(x_raw),
          static_cast<double>(y_raw),
          static_cast<double>(z_raw)
        };
        break;
      }
      default:
        break;
    }
  }

  std::array<double, 4> quaternionFromEuler(
    const double roll_rad,
    const double pitch_rad,
    const double yaw_rad) const
  {
    const double half_roll = roll_rad * 0.5;
    const double half_pitch = pitch_rad * 0.5;
    const double half_yaw = yaw_rad * 0.5;

    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);

    return {
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy
    };
  }

  void publishImu()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = frame_start_time_;
    msg.header.frame_id = frame_id_;

    constexpr double kAccelBiasX = 0.08782286804851495;
    constexpr double kAccelBiasY = -0.08779280747803357;

    msg.linear_acceleration.x = state_.acceleration[0] - kAccelBiasX;
    msg.linear_acceleration.y = (-state_.acceleration[1]) - kAccelBiasY;
    msg.linear_acceleration.z = state_.acceleration[2];
    msg.linear_acceleration_covariance[0] = 4.667e-05;
    msg.linear_acceleration_covariance[4] = 1.468e-04;
    msg.linear_acceleration_covariance[8] = 8.595e-05;

    msg.angular_velocity.x = state_.angular_velocity[0];
    msg.angular_velocity.y = -state_.angular_velocity[1];
    msg.angular_velocity.z = state_.angular_velocity[2];
    msg.angular_velocity_covariance[0] = kAngularVelocityVariance;
    msg.angular_velocity_covariance[4] = kAngularVelocityVariance;
    msg.angular_velocity_covariance[8] = kAngularVelocityVariance;

    const double roll_rad = state_.angle_degree[0] * kPi / 180.0;
    const double pitch_rad = -state_.angle_degree[1] * kPi / 180.0;
    const double yaw_rad = state_.angle_degree[2] * kPi / 180.0;
    const auto quaternion = quaternionFromEuler(roll_rad, pitch_rad, yaw_rad);

    msg.orientation.x = quaternion[0];
    msg.orientation.y = quaternion[1];
    msg.orientation.z = quaternion[2];
    msg.orientation.w = quaternion[3];
    msg.orientation_covariance[0] = kOrientationRollPitchVariance;
    msg.orientation_covariance[4] = kOrientationRollPitchVariance;
    msg.orientation_covariance[8] = kOrientationYawVariance;

    imu_pub_->publish(msg);
  }

  std::string port_name_;
  int baud_rate_ {};
  std::string frame_id_;
  bool disable_magnetometer_ {false};
  bool zero_reference_at_startup_ {true};
  int serial_fd_ {-1};
  std::thread read_thread_;
  std::atomic<bool> running_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::array<uint8_t, kFrameLength> frame_ {};
  std::size_t frame_index_ {0};
  rclcpp::Time frame_start_time_;
  ImuState state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WildbotImuNode>());
  rclcpp::shutdown();
  return 0;
}

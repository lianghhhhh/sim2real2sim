#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

// Wire protocol between the hardware plugin and the ESP32 wheel controller
// (wildbot_wheelctrl), plus the unit conversions for DDSM115 feedback.
// Nothing here depends on ROS, so it can be unit tested on its own.

namespace kros_car::wheel
{

// The firmware polls motor IDs 1..kNumWheels; every array is indexed by id - 1.
constexpr std::size_t kNumWheels = 4;

constexpr int kModeSpeedLoop = 2;

// DDSM115: torque current -32767..32767 <-> -8..8 A.
constexpr int kRawFullScale = 32767;
constexpr double kCurrentFullScaleA = 8.0;

// DDSM115: position 0..32767 <-> one revolution.
constexpr int kPositionModulus = 32768;

// The DDSM115 position count runs against its own speed and torque current:
// with a positive speed the count goes down. Measured on the car with the
// wheels off the ground (scripts/wheel_bench.py, 2026-10-20: d(position)/dt =
// -0.99 x speed on all four motors). Speed and current need no correction.
constexpr int kPositionSign = -1;

// Error code bits of the DDSM115 feedback.
constexpr int kErrSensor = 0x01;
constexpr int kErrOvercurrent = 0x02;
constexpr int kErrPhaseOvercurrent = 0x04;
constexpr int kErrStall = 0x08;
constexpr int kErrTroubleshooting = 0x10;

// One FB_MOTOR_ALL (T:20012) line from the firmware.
struct Frame
{
  uint32_t esp_ms{0};   // ESP32 millis() when the line was sent
  unsigned ok_mask{0};  // bit i: wheel id i+1 answered in this cycle
  std::array<int, kNumWheels> mode{};
  std::array<int, kNumWheels> spd_rpm{};
  std::array<int, kNumWheels> pos_raw{};
  std::array<int, kNumWheels> tor_raw{};
  std::array<int, kNumWheels> err{};

  bool ok(std::size_t index) const {return ((ok_mask >> index) & 1u) != 0u;}
};

// {"T":10011,"cmd":[c1,c2,c3,c4],"act":a}\n  -- targets for wheel id 1..4.
std::string build_command_all(const std::array<int, kNumWheels> & cmd, int act);

// {"T":11001,"time":ms}\n
std::string build_heartbeat(int timeout_ms);

// {"T":10012,"id":id,"mode":mode}\n
std::string build_change_mode(int motor_id, int mode);

// Returns false for anything that is not a well-formed T:20012 line with
// every value in range, so boot noise and other message types are ignored.
bool parse_frame(const std::string & line, Frame & frame);

// True for the firmware's heartbeat-stop notice ({"T":20011,"hb":0}).
bool is_heartbeat_stop(const std::string & line);

double rpm_to_rad_per_s(double rpm);
double raw_to_ampere(int raw);
double counts_to_rad(long long counts);
// Unwrapped position count -> rad, in the sign of the motor's speed.
double motor_position_to_rad(long long counts);

// "sensor,stall" style description of a DDSM115 error code.
std::string describe_error(int err);

// Turns the single-turn position (0..32767) into a continuous count.
// Valid as long as the wheel turns less than half a revolution between two
// readings: at 50 Hz that is 1500 rpm, far above what the motor can do.
class PositionUnwrapper
{
public:
  // Returns the counts accumulated since the first reading.
  long long update(int raw);
  void reset();

private:
  bool has_last_{false};
  int last_raw_{0};
  long long counts_{0};
};

// Maps the ESP32 clock onto the host clock.
//
// A line arrives after a delay that jitters by several milliseconds (USB
// buffering, scheduling), so the receive time is a noisy timestamp. The
// ESP32's own millis() is steady, and the receive time can never be earlier
// than the send time, so the smallest (receive - esp) difference seen is the
// best estimate of the clock offset.
class ClockMapper
{
public:
  // Returns the estimated host time [s] at which the frame was sent. Never
  // later than host_rx_s.
  double update(uint32_t esp_ms, double host_rx_s);
  void reset();

private:
  // The offset may grow this fast to follow crystal drift (typically < 50 ppm).
  static constexpr double kMaxDriftRate = 2e-4;
  // A longer gap, or the ESP32 counter going backwards, means a reboot or a
  // dropped link: start over.
  static constexpr uint32_t kResetGapMs = 1000;

  bool initialized_{false};
  uint32_t last_esp_ms_{0};
  double last_host_rx_s_{0.0};
  double esp_s_{0.0};
  double offset_s_{0.0};
};

}  // namespace kros_car::wheel

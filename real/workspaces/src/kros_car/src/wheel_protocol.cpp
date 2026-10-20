#include "kros_car/wheel_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <sstream>
#include <utility>

namespace kros_car::wheel
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Returns the index right after `"key":`, or npos.
std::size_t find_value(const std::string & line, const char * key)
{
  const std::string needle = std::string("\"") + key + "\":";
  const std::size_t pos = line.find(needle);
  return pos == std::string::npos ? std::string::npos : pos + needle.size();
}

bool read_int(const std::string & line, std::size_t & pos, long long & out)
{
  const char * begin = line.c_str() + pos;
  char * end = nullptr;
  errno = 0;
  const long long value = std::strtoll(begin, &end, 10);
  if (end == begin || errno == ERANGE) {
    return false;
  }
  out = value;
  pos += static_cast<std::size_t>(end - begin);
  return true;
}

bool read_scalar(const std::string & line, const char * key, long long min, long long max,
  long long & out)
{
  std::size_t pos = find_value(line, key);
  if (pos == std::string::npos) {
    return false;
  }
  return read_int(line, pos, out) && out >= min && out <= max;
}

bool read_array(const std::string & line, const char * key, long long min, long long max,
  std::array<int, kNumWheels> & out)
{
  std::size_t pos = find_value(line, key);
  if (pos == std::string::npos || pos >= line.size() || line[pos] != '[') {
    return false;
  }
  ++pos;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    long long value = 0;
    if (!read_int(line, pos, value) || value < min || value > max) {
      return false;
    }
    out[i] = static_cast<int>(value);
    const char expected = (i + 1 < kNumWheels) ? ',' : ']';
    if (pos >= line.size() || line[pos] != expected) {
      return false;
    }
    ++pos;
  }
  return true;
}

}  // namespace

std::string build_command_all(const std::array<int, kNumWheels> & cmd, int act)
{
  std::ostringstream oss;
  oss << "{\"T\":10011,\"cmd\":[";
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    oss << (i == 0 ? "" : ",") << cmd[i];
  }
  oss << "],\"act\":" << act << "}\n";
  return oss.str();
}

std::string build_heartbeat(int timeout_ms)
{
  std::ostringstream oss;
  oss << "{\"T\":11001,\"time\":" << timeout_ms << "}\n";
  return oss.str();
}

std::string build_change_mode(int motor_id, int mode)
{
  std::ostringstream oss;
  oss << "{\"T\":10012,\"id\":" << motor_id << ",\"mode\":" << mode << "}\n";
  return oss.str();
}

bool parse_frame(const std::string & line, Frame & frame)
{
  // Trim the trailing CR / whitespace the firmware's println leaves behind.
  std::size_t end = line.size();
  while (end > 0 && (line[end - 1] == '\r' || line[end - 1] == '\n' || line[end - 1] == ' ')) {
    --end;
  }
  if (end < 2 || line[0] != '{' || line[end - 1] != '}') {
    return false;
  }

  long long type = 0;
  long long esp_ms = 0;
  long long ok_mask = 0;
  Frame parsed;
  if (!read_scalar(line, "T", 20012, 20012, type) ||
    !read_scalar(line, "ms", 0, 0xFFFFFFFFLL, esp_ms) ||
    !read_scalar(line, "ok", 0, (1LL << kNumWheels) - 1, ok_mask) ||
    !read_array(line, "mode", 0, 255, parsed.mode) ||
    !read_array(line, "spd", -32768, 32767, parsed.spd_rpm) ||
    !read_array(line, "pos", 0, kPositionModulus - 1, parsed.pos_raw) ||
    !read_array(line, "tor", -32768, 32767, parsed.tor_raw) ||
    !read_array(line, "err", 0, 255, parsed.err))
  {
    return false;
  }

  parsed.esp_ms = static_cast<uint32_t>(esp_ms);
  parsed.ok_mask = static_cast<unsigned>(ok_mask);
  frame = parsed;
  return true;
}

bool is_heartbeat_stop(const std::string & line)
{
  return line.find("\"T\":20011") != std::string::npos &&
         line.find("\"hb\":0") != std::string::npos;
}

double rpm_to_rad_per_s(double rpm)
{
  return rpm * 2.0 * kPi / 60.0;
}

double raw_to_ampere(int raw)
{
  return static_cast<double>(raw) * kCurrentFullScaleA / static_cast<double>(kRawFullScale);
}

double counts_to_rad(long long counts)
{
  return static_cast<double>(counts) * 2.0 * kPi / static_cast<double>(kPositionModulus);
}

double motor_position_to_rad(long long counts)
{
  return kPositionSign * counts_to_rad(counts);
}

std::string describe_error(int err)
{
  static const std::array<std::pair<int, const char *>, 5> kBits = {{
    {kErrSensor, "sensor"},
    {kErrOvercurrent, "overcurrent"},
    {kErrPhaseOvercurrent, "phase_overcurrent"},
    {kErrStall, "stall"},
    {kErrTroubleshooting, "troubleshooting"},
  }};

  std::string text;
  for (const auto & [bit, name] : kBits) {
    if ((err & bit) != 0) {
      text += (text.empty() ? "" : ",");
      text += name;
    }
  }
  return text.empty() ? "unknown" : text;
}

long long PositionUnwrapper::update(int raw)
{
  if (!has_last_) {
    has_last_ = true;
    last_raw_ = raw;
    return counts_;
  }

  int delta = raw - last_raw_;
  if (delta > kPositionModulus / 2) {
    delta -= kPositionModulus;
  } else if (delta < -kPositionModulus / 2) {
    delta += kPositionModulus;
  }
  counts_ += delta;
  last_raw_ = raw;
  return counts_;
}

void PositionUnwrapper::reset()
{
  has_last_ = false;
  last_raw_ = 0;
  counts_ = 0;
}

double ClockMapper::update(uint32_t esp_ms, double host_rx_s)
{
  if (initialized_) {
    // Unsigned subtraction survives the 49-day wrap of millis(); a counter
    // that went backwards shows up as a huge delta.
    const uint32_t delta_ms = esp_ms - last_esp_ms_;
    const double host_gap_s = host_rx_s - last_host_rx_s_;
    if (delta_ms > kResetGapMs || host_gap_s < 0.0 || host_gap_s > kResetGapMs * 1e-3) {
      initialized_ = false;
    } else {
      const double delta_s = delta_ms * 1e-3;
      esp_s_ += delta_s;
      offset_s_ = std::min(host_rx_s - esp_s_, offset_s_ + kMaxDriftRate * delta_s);
    }
  }

  if (!initialized_) {
    initialized_ = true;
    esp_s_ = 0.0;
    offset_s_ = host_rx_s;
  }

  last_esp_ms_ = esp_ms;
  last_host_rx_s_ = host_rx_s;
  return esp_s_ + offset_s_;
}

void ClockMapper::reset()
{
  initialized_ = false;
}

}  // namespace kros_car::wheel

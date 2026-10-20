#include "kros_car/kros_car_system.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace kros_car
{

std::vector<std::string> KrosCarSystem::split_csv(const std::string & raw)
{
  std::vector<std::string> values;
  std::stringstream ss(raw);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char ch) {
      return !std::isspace(ch);
    }));
    item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char ch) {
      return !std::isspace(ch);
    }).base(), item.end());
    if (!item.empty()) {
      values.push_back(item);
    }
  }
  return values;
}

int KrosCarSystem::parse_int_or_default(const std::string & value, int fallback)
{
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

double KrosCarSystem::parse_double_or_default(const std::string & value, double fallback)
{
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

bool KrosCarSystem::parse_bool_or_default(const std::string & value, bool fallback)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (lowered == "true" || lowered == "1") {
    return true;
  }
  if (lowered == "false" || lowered == "0") {
    return false;
  }
  return fallback;
}

bool KrosCarSystem::set_baud_rate(LibSerial::SerialPort & port, int baudrate)
{
  switch (baudrate) {
    case 9600:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_9600); break;
    case 19200:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_19200); break;
    case 38400:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_38400); break;
    case 57600:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_57600); break;
    case 115200:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_115200); break;
    case 230400:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_230400); break;
    case 460800:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_460800); break;
    case 921600:
      port.SetBaudRate(LibSerial::BaudRate::BAUD_921600); break;
    default:
      return false;
  }
  return true;
}

}  // namespace kros_car

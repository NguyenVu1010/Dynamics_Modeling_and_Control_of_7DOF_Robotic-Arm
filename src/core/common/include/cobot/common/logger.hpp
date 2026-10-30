#pragma once

#include <string_view>

// Core logger interface. ros2_node adapts it to rclcpp; bench and tests use the
// default stderr sink. Never log per point in the hot path.
namespace cobot::common {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

class Logger {
 public:
  virtual ~Logger() = default;
  virtual void log(LogLevel level, std::string_view msg) = 0;
};

// Process-wide sink. Owner: the host (bench or ros2_node) sets it once before starting
// threads; it is not safe to swap while other threads log.
void set_logger(Logger* logger);
Logger& logger();

void log_debug(std::string_view msg);
void log_info(std::string_view msg);
void log_warn(std::string_view msg);
void log_error(std::string_view msg);

}  // namespace cobot::common

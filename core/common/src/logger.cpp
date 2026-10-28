#include "cobot/common/logger.hpp"

#include <cstdio>

namespace cobot::common {
namespace {

class StderrLogger final : public Logger {
 public:
  void log(LogLevel level, std::string_view msg) override {
    const char* tag = "INFO";
    switch (level) {
      case LogLevel::kDebug:
        tag = "DEBUG";
        break;
      case LogLevel::kInfo:
        tag = "INFO";
        break;
      case LogLevel::kWarn:
        tag = "WARN";
        break;
      case LogLevel::kError:
        tag = "ERROR";
        break;
    }
    std::fprintf(stderr, "[%s] %.*s\n", tag, static_cast<int>(msg.size()), msg.data());
  }
};

StderrLogger g_default_logger;
Logger* g_logger = &g_default_logger;

}  // namespace

void set_logger(Logger* logger) {
  g_logger = logger != nullptr ? logger : &g_default_logger;
}
Logger& logger() {
  return *g_logger;
}

void log_debug(std::string_view msg) {
  g_logger->log(LogLevel::kDebug, msg);
}
void log_info(std::string_view msg) {
  g_logger->log(LogLevel::kInfo, msg);
}
void log_warn(std::string_view msg) {
  g_logger->log(LogLevel::kWarn, msg);
}
void log_error(std::string_view msg) {
  g_logger->log(LogLevel::kError, msg);
}

}  // namespace cobot::common

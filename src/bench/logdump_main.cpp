// cobot_logdump <controller> [--out <csv>] [--keep] [--remove] (feature blackbox-log, ADR-020):
// appends the rows of the black-box region "/cobot_log_<controller>" that were never exported to
// a Run CSV (default ./<controller>-<YYYYmmdd-HHMMSS>.csv) and prints "<rows> rows -> <csv>"
// (", <lost> rows lost" when rows were overwritten before any export). --keep leaves the rows
// unexported, --remove removes the region afterwards. Exit 0 done, 1 no region or nothing to
// export, 2 bad arguments, layout mismatch or a write error (reason on stderr). No ROS.
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>

#include "cobot/pipeline/recorder_shm.hpp"

namespace {

constexpr int kDof = 7;
constexpr int kExitNothing = 1;
constexpr int kExitBad = 2;

const char* const kUsage = "usage: cobot_logdump <controller> [--out <csv>] [--keep] [--remove]\n";

std::string default_out(const std::string& controller) {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&now, &tm);
  char buf[32] = {};
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm);
  return "./" + controller + "-" + buf + ".csv";
}

int bad(const std::string& why) {
  std::fprintf(stderr, "cobot_logdump: %s\n%s", why.c_str(), kUsage);
  return kExitBad;
}

}  // namespace

int main(int argc, char** argv) {
  using Storage = cobot::pipeline::ShmStorage<kDof>;
  std::string controller;
  std::string out;
  bool keep = false;
  bool remove = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--keep") {
      keep = true;
    } else if (arg == "--remove") {
      remove = true;
    } else if (arg == "--out") {
      if (i + 1 >= argc) {
        return bad("--out needs a file");
      }
      out = argv[++i];
    } else if (arg.rfind("--", 0) == 0 || !controller.empty()) {
      return bad("unexpected argument '" + arg + "'");
    } else {
      controller = arg;
    }
  }
  if (controller.empty()) {
    return bad("no controller name");
  }
  if (out.empty()) {
    out = default_out(controller);
  }
  if (!Storage::exists(controller)) {
    std::fprintf(stderr, "cobot_logdump: no region /cobot_log_%s\n", controller.c_str());
    return kExitNothing;
  }
  cobot::common::Result<Storage> storage = Storage::open_existing(controller);
  if (!storage) {
    std::fprintf(stderr, "cobot_logdump: %s\n", storage.reason().c_str());
    return kExitBad;
  }
  const cobot::common::Result<cobot::pipeline::DumpResult> dumped =
      storage.value().export_unexported(out, cobot::pipeline::ShmExport::kAll, keep);
  if (remove) {
    const cobot::common::Status removed = Storage::remove(controller);
    if (!removed) {
      std::fprintf(stderr, "cobot_logdump: %s\n", removed.reason().c_str());
    }
  }
  if (!dumped) {
    std::fprintf(stderr, "cobot_logdump: %s\n", dumped.reason().c_str());
    return kExitBad;
  }
  const cobot::pipeline::DumpResult& r = dumped.value();
  if (r.rows == 0) {
    std::fprintf(stderr, "cobot_logdump: nothing to export in /cobot_log_%s\n", controller.c_str());
    return kExitNothing;
  }
  std::printf("%zu rows -> %s", r.rows, r.csv.c_str());
  if (r.lost > 0) {
    std::printf(", %llu rows lost", static_cast<unsigned long long>(r.lost));
  }
  std::printf("\n");
  return 0;
}

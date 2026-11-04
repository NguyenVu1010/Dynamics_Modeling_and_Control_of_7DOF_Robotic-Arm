// cobot_bench serve + cobot_bridge_cli (feature controller-bridge, Part A): bench::serve runs the
// paced loop and the bridge on threads of this test (config/default.yaml + ideal profile, region
// /cobot_cobot_test_serve_<pid>); the headless client runs as a child process from the repository
// root. Pinned:
//   - a full session (claim, wait-standby, jog, move-joint, move-tcp, waypoints, export, release)
//     exits 0 with one "<k> <step>: ok ..." line per step; the exported segment has the Run CSV
//     header bytes, ends in standby and holds only `waypoints` rows (the last motion step);
//   - a second client's claim while the first holds authority: exit 1, "held by pid";
//   - an unknown step: exit 2; a name without a region: exit 2, "no bridge";
//   - `cobot_bench serve --duration-s 1` exits 0, prints "serving /cobot_<name>" and leaves no
//     region behind.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/kinematics/kinematics.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "serve.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

constexpr int kN = 7;
const char* const kBenchExe = COBOT_BENCH_EXE;
const char* const kBridgeCliExe = COBOT_BRIDGE_CLI_EXE;

fs::path repo_path(const std::string& rel) {
  return fs::path(COBOT_REPO_DIR) / rel;
}

std::string pid_text() {
  return std::to_string(::getpid());
}

std::string quoted(const std::string& s) {
  return "'" + s + "'";
}

std::string read_file(const fs::path& path) {
  const std::ifstream in(path, std::ios::binary);
  std::ostringstream os;
  os << in.rdbuf();
  return os.str();
}

std::vector<std::string> lines_of(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

fs::path test_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bridge_serve_cli" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

bool region_exists(const std::string& name) {
  return fs::exists(fs::path("/dev/shm") / ("cobot_" + name));
}

struct Cli {
  int exit_code = -1;
  std::string out;  // stdout and stderr
};

// Runs `exe args` from the repository root (the waypoint path of the steps is relative to it).
Cli run_tool(const char* exe, const std::string& args) {
  Cli cli;
  if (std::string(exe).empty()) {
    ADD_FAILURE() << "tool not built (empty path)";
    return cli;
  }
  const std::string cmd = "cd " + quoted(repo_path("").string()) + " && " +
                          quoted(std::string(exe)) + " " + args + " 2>&1";
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    ADD_FAILURE() << "popen failed: " << cmd;
    return cli;
  }
  std::array<char, 4096> buf{};
  std::size_t n = 0;
  while ((n = std::fread(buf.data(), 1, buf.size(), pipe)) > 0) {
    cli.out.append(buf.data(), n);
  }
  const int status = pclose(pipe);
  cli.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return cli;
}

// config/default.yaml with absolute paths and a 5 s Recorder per buffer (instead of 60 s: two
// buffers of 60 000 rows would make this test touch ~120 MB; the bridge exports every 200 ms).
fs::path serve_config(const fs::path& dir) {
  std::string text = read_file(repo_path("config/default.yaml"));
  const auto replace = [&text](const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) {
      text.replace(at, from.size(), to);
    }
  };
  replace("urdf_path: ../src/sim/urdf/cobot30_5.urdf",
          "urdf_path: " + repo_path("src/sim/urdf/cobot30_5.urdf").string());
  replace("output_dir: ../results", "output_dir: " + (dir / "results").string());
  replace("capacity_s: 60.0", "capacity_s: 5.0");
  const fs::path yaml = dir / "serve_config.yaml";
  std::ofstream(yaml, std::ios::binary) << text;
  return yaml;
}

// R = Rz(yaw) Ry(pitch) Rx(roll), element by element.
common::Mat3 rpy_matrix(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  common::Mat3 r;
  r << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,  //
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,   //
      -sp, cp * sr, cp * cr;
  return r;
}

// Roll, pitch, yaw of R = Rz(yaw) Ry(pitch) Rx(roll). At pitch = +-90 deg only yaw -+ roll is
// defined: roll = 0 and yaw = atan2(-R01, R11) (row 0 = [0, -sin(yaw +- roll), ...],
// row 1 = [0, cos(yaw +- roll), ...] there).
common::Vec3 rpy_of(const common::Mat3& R) {
  const double c = std::hypot(R(0, 0), R(1, 0));
  const double pitch = std::atan2(-R(2, 0), c);
  if (c > 1e-9) {
    return {std::atan2(R(2, 1), R(2, 2)), pitch, std::atan2(R(1, 0), R(0, 0))};
  }
  return {0.0, pitch, std::atan2(-R(0, 1), R(1, 1))};
}

// The home TCP pose of default.yaml (FK of home_q, tcp_offset applied).
std::optional<common::Pose> home_tcp() {
  const auto cfg = pipeline::load_config<kN>(repo_path("config/default.yaml"));
  if (!cfg) {
    ADD_FAILURE() << cfg.reason();
    return std::nullopt;
  }
  const auto chain =
      model::parse_urdf_chain(cfg.value().urdf_path, cfg.value().base_link, cfg.value().tip_link);
  if (!chain) {
    ADD_FAILURE() << chain.reason();
    return std::nullopt;
  }
  const auto arm = model::make_model<kN>(chain.value(), cfg.value().gravity_b);
  if (!arm) {
    ADD_FAILURE() << arm.reason();
    return std::nullopt;
  }
  kinematics::FkState<kN> fk;
  kinematics::forward_kinematics(arm.value(), cfg.value().home_q_rad, fk);
  return common::compose(fk.T_b_e, cfg.value().tcp_offset.T_tip_tcp);
}

std::string fixed(double v) {
  std::array<char, 64> buf{};
  std::snprintf(buf.data(), buf.size(), "%.9f", v);
  return buf.data();
}

// `move-tcp:x,y,z,roll_deg,pitch_deg,yaw_deg` of the home TCP moved by dx along base x.
std::string move_tcp_step(const common::Pose& tcp, double dx) {
  const common::Vec3 rpy = rpy_of(tcp.R);
  const common::Mat3 back = rpy_matrix(rpy.x(), rpy.y(), rpy.z());
  EXPECT_LT((back - tcp.R).cwiseAbs().maxCoeff(), 1e-9) << "rpy_of does not invert rpy_matrix";
  const double deg = 180.0 / 3.14159265358979323846;
  return "move-tcp:" + fixed(tcp.p.x() + dx) + "," + fixed(tcp.p.y()) + "," + fixed(tcp.p.z()) +
         "," + fixed(rpy.x() * deg) + "," + fixed(rpy.y() * deg) + "," + fixed(rpy.z() * deg);
}

int ok_lines(const std::string& out) {
  static const std::regex ok_line(R"(^[0-9]+ \S+: ok( .*)?$)");
  int n = 0;
  for (const std::string& line : lines_of(out)) {
    if (std::regex_match(line, ok_line)) {
      ++n;
    }
  }
  return n;
}

// bench::serve on a thread of the test for each test, stopped in TearDown.
class BridgeServeCli : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string test = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    dir_ = test_dir(test);
    options_.config_yaml = serve_config(dir_);
    options_.imperfections_yaml = repo_path("config/imperfections/ideal.yaml");
    options_.name = "cobot_test_serve_" + pid_text();
    options_.out_dir = dir_ / "serve";
    thread_ = std::thread([this] {
      const common::Status st = serve(options_, stop_, [this](const std::string& region) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ready_ = region;
        cv_.notify_all();
      });
      const std::lock_guard<std::mutex> lock(mutex_);
      result_ = st;
      cv_.notify_all();
    });
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::seconds(60), [this] { return ready_ || result_; });
    ASSERT_TRUE(ready_) << "serve never became ready"
                        << (result_ ? ": " + result_->reason() : std::string());
    EXPECT_NE(ready_->find(options_.name), std::string::npos) << *ready_;
    EXPECT_TRUE(region_exists(options_.name));
  }

  void TearDown() override {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
    ASSERT_TRUE(result_);
    EXPECT_TRUE(result_->is_ok()) << result_->reason();
    EXPECT_FALSE(region_exists(options_.name)) << "serve left its region behind";
    EXPECT_TRUE(fs::exists(options_.out_dir / "run.csv"));
  }

  const std::string& name() const { return options_.name; }
  const fs::path& dir() const { return dir_; }

 private:
  fs::path dir_;
  ServeOptions options_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<std::string> ready_;
  std::optional<common::Status> result_;
};

TEST_F(BridgeServeCli, FullSessionRunsEveryStepAndExportsTheWaypointSegment) {
  const auto tcp = home_tcp();
  ASSERT_TRUE(tcp);
  const fs::path segment = dir() / "segment.csv";
  // home_q_deg of default.yaml.
  const std::vector<std::string> steps = {
      "claim",
      "wait-standby",
      "jog-joint:1:+:0.3",
      "move-joint:0,30,0,-90,0,60,0",
      move_tcp_step(*tcp, 0.05),
      "waypoints:tests/e2e/waypoints/e2e_13.yaml:2",
      "export:" + segment.string(),
      "release",
  };
  std::string args = name();
  for (const std::string& s : steps) {
    args += " " + s;
  }
  const Cli cli = run_tool(kBridgeCliExe, args);
  EXPECT_EQ(cli.exit_code, 0) << cli.out;
  EXPECT_EQ(ok_lines(cli.out), static_cast<int>(steps.size())) << cli.out;

  ASSERT_TRUE(fs::exists(segment)) << cli.out;
  const std::vector<std::string> rows = lines_of(read_file(segment));
  ASSERT_GE(rows.size(), 2U);
  std::ostringstream header;
  pipeline::write_csv_header<kN>(header);
  EXPECT_EQ(rows.front() + "\n", header.str());
  for (std::size_t i = 1; i < rows.size(); ++i) {
    const std::string& row = rows[i];
    ASSERT_EQ(row.substr(row.rfind(',') + 1), "waypoints") << "row " << i << ": " << row;
  }
  const std::string& last = rows.back();
  const std::size_t c1 = last.find(',');
  ASSERT_NE(c1, std::string::npos);
  EXPECT_EQ(last.substr(c1 + 1, last.find(',', c1 + 1) - c1 - 1), "standby") << last;
}

TEST_F(BridgeServeCli, SecondClientIsRefusedWhileTheFirstHoldsAuthority) {
  Cli first;
  std::thread holder([&] { first = run_tool(kBridgeCliExe, name() + " claim pause:3"); });
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const Cli second = run_tool(kBridgeCliExe, name() + " claim");
  holder.join();
  EXPECT_EQ(second.exit_code, 1) << second.out;
  EXPECT_NE(second.out.find("held by pid"), std::string::npos) << second.out;
  EXPECT_EQ(first.exit_code, 0) << first.out;
  EXPECT_EQ(ok_lines(first.out), 2) << first.out;
}

TEST_F(BridgeServeCli, UnknownStepIsABadArgument) {
  const Cli cli = run_tool(kBridgeCliExe, name() + " claim frobnicate");
  EXPECT_EQ(cli.exit_code, 2) << cli.out;
}

TEST_F(BridgeServeCli, NameWithoutARegionIsNoBridge) {
  const Cli cli = run_tool(kBridgeCliExe, "cobot_test_nobridge_" + pid_text() + " claim");
  EXPECT_EQ(cli.exit_code, 2) << cli.out;
  EXPECT_NE(cli.out.find("no bridge"), std::string::npos) << cli.out;
}

TEST(BridgeServeCliTool, ServeFromTheCommandLineEndsAfterItsDurationWithoutARegion) {
  const fs::path dir = test_dir("serve_cli");
  const std::string name = "cobot_test_serve_cli_" + pid_text();
  const fs::path out = dir / "out";
  const Cli cli =
      run_tool(kBenchExe,
               "serve --config " + quoted(serve_config(dir).string()) + " --imperfections " +
                   quoted(repo_path("config/imperfections/ideal.yaml").string()) + " --name " +
                   name + " --out " + quoted(out.string()) + " --duration-s 1");
  EXPECT_EQ(cli.exit_code, 0) << cli.out;
  EXPECT_NE(cli.out.find("serving /cobot_" + name), std::string::npos) << cli.out;
  EXPECT_FALSE(region_exists(name));
  EXPECT_TRUE(fs::exists(out / "run.csv")) << cli.out;
}

}  // namespace
}  // namespace cobot::bench

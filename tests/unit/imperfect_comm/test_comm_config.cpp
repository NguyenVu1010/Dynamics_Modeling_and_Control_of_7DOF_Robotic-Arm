#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

// Loader and sweep side of feature comm-timing (realism-2 design 6.3): the optional `comm` group
// (servo_rate_hz, packet_loss_prob, latency_burst_prob, burst_cycles; all 0 = off), the bench-only
// timing keys overrun_prob and overrun_cycles, range errors naming the full key, the new
// hardware.yaml params, and config.<section>.<key> overrides of config/default.yaml
// (pipeline::load_config_with; only config.safety.hal_timeout_ms must be supported).
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;

std::string ideal_text() {
  const std::ifstream in(fs::path(COBOT_REPO_DIR) / "config/imperfections/ideal.yaml",
                         std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// `text` without the group `  <group>:` (and every deeper-indented line under it).
std::string without_group(std::string text, const std::string& group) {
  const std::size_t start = text.find("\n  " + group + ":");
  if (start == std::string::npos) {
    return text;
  }
  std::size_t end = text.find('\n', start + 1);
  while (end != std::string::npos && text.compare(end, 5, "\n    ") == 0) {
    end = text.find('\n', end + 1);
  }
  text.erase(start, (end == std::string::npos ? text.size() : end) - start);
  return text;
}

// `text` with the group replaced by (or extended with) the given `key: value` lines; the group
// is the last one of `imperfections` (the only top-level key of ideal.yaml).
std::string with_group(const std::string& text,
                       const std::string& group,
                       const std::vector<std::pair<std::string, std::string>>& keys) {
  std::string out = without_group(text, group);
  while (!out.empty() && out.back() == '\n') {
    out.pop_back();
  }
  out.append("\n  ").append(group).append(":\n    source: \"test\"\n");
  for (const auto& [key, value] : keys) {
    out.append("    ").append(key).append(": ").append(value).append("\n");
  }
  return out;
}

std::vector<std::pair<std::string, std::string>> comm_keys(const std::string& rate,
                                                           const std::string& loss,
                                                           const std::string& burst_prob,
                                                           const std::string& burst_cycles) {
  return {{"servo_rate_hz", rate},
          {"packet_loss_prob", loss},
          {"latency_burst_prob", burst_prob},
          {"burst_cycles", burst_cycles}};
}

Result<imperfect::ImperfectionConfig<7>> load_text(const std::string& name,
                                                   const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "imperfect_comm";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return load_imperfections<7>(p);
}

void expect_rejected(const std::string& name, const std::string& text, const std::string& key) {
  const auto r = load_text(name, text);
  ASSERT_FALSE(r.is_ok()) << name;
  EXPECT_NE(r.reason().find(key), std::string::npos) << r.reason();
}

TEST(CommConfig, IdealProfileHasCommAndOverrunOff) {
  const auto r = load_text("ideal.yaml", ideal_text());
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const imperfect::CommConfig& c = r.value().comm;
  EXPECT_EQ(c.servo_rate_hz, 0.0);
  EXPECT_EQ(c.packet_loss_prob, 0.0);
  EXPECT_EQ(c.latency_burst_prob, 0.0);
  EXPECT_EQ(c.burst_cycles, 0);
  EXPECT_EQ(r.value().timing.overrun_prob, 0.0);
  EXPECT_EQ(r.value().timing.overrun_cycles, 0);
}

TEST(CommConfig, CommGroupIsOptional) {
  const auto r = load_text("no_comm.yaml", without_group(ideal_text(), "comm"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().comm.servo_rate_hz, 0.0);
  EXPECT_EQ(r.value().comm.packet_loss_prob, 0.0);
  EXPECT_EQ(r.value().comm.latency_burst_prob, 0.0);
  EXPECT_EQ(r.value().comm.burst_cycles, 0);
}

TEST(CommConfig, KeysAreRead) {
  std::string t = with_group(ideal_text(), "comm", comm_keys("250", "1.0e-3", "1.0e-4", "5"));
  t = with_group(
      t,
      "timing",
      {{"period_jitter_std_us", "100"}, {"overrun_prob", "1.0e-3"}, {"overrun_cycles", "3"}});
  const auto r = load_text("comm.yaml", t);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const imperfect::CommConfig& c = r.value().comm;
  EXPECT_EQ(c.servo_rate_hz, 250.0);
  EXPECT_EQ(c.packet_loss_prob, 1.0e-3);
  EXPECT_EQ(c.latency_burst_prob, 1.0e-4);
  EXPECT_EQ(c.burst_cycles, 5);
  EXPECT_EQ(r.value().timing.period_jitter_std_us, 100.0);
  EXPECT_EQ(r.value().timing.overrun_prob, 1.0e-3);
  EXPECT_EQ(r.value().timing.overrun_cycles, 3);
}

TEST(CommConfig, BoundaryValuesAreAccepted) {
  // Probabilities 0 and 1, burst_cycles at kMaxDelayCycles = 32.
  const auto r =
      load_text("bounds.yaml", with_group(ideal_text(), "comm", comm_keys("500", "1", "0", "32")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().comm.packet_loss_prob, 1.0);
  EXPECT_EQ(r.value().comm.burst_cycles, 32);
}

TEST(CommConfig, InvalidValuesRejectedNamingTheKey) {
  const std::string t = ideal_text();
  expect_rejected("loss_gt1.yaml",
                  with_group(t, "comm", comm_keys("0", "1.5", "0", "0")),
                  "comm.packet_loss_prob");
  expect_rejected("loss_neg.yaml",
                  with_group(t, "comm", comm_keys("0", "-0.1", "0", "0")),
                  "comm.packet_loss_prob");
  expect_rejected("burst_gt1.yaml",
                  with_group(t, "comm", comm_keys("0", "0", "2", "1")),
                  "comm.latency_burst_prob");
  expect_rejected("burst_cycles_33.yaml",
                  with_group(t, "comm", comm_keys("0", "0", "0", "33")),
                  "comm.burst_cycles");
  expect_rejected("burst_cycles_neg.yaml",
                  with_group(t, "comm", comm_keys("0", "0", "0", "-1")),
                  "comm.burst_cycles");
  expect_rejected("rate_neg.yaml",
                  with_group(t, "comm", comm_keys("-250", "0", "0", "0")),
                  "comm.servo_rate_hz");
  std::vector<std::pair<std::string, std::string>> unknown = comm_keys("0", "0", "0", "0");
  unknown.emplace_back("packet_loss_percent", "1");
  expect_rejected("unknown.yaml", with_group(t, "comm", unknown), "packet_loss_percent");
  expect_rejected("overrun_prob_gt1.yaml",
                  with_group(t, "timing", {{"period_jitter_std_us", "0"}, {"overrun_prob", "1.5"}}),
                  "timing.overrun_prob");
  expect_rejected(
      "overrun_cycles_neg.yaml",
      with_group(t, "timing", {{"period_jitter_std_us", "0"}, {"overrun_cycles", "-1"}}),
      "timing.overrun_cycles");
}

TEST(CommConfig, HardwareSweepHasTheCommParams) {
  const auto sweep = load_sweep(fs::path(COBOT_REPO_DIR) / "config/sweeps/hardware.yaml");
  ASSERT_TRUE(sweep.is_ok()) << sweep.reason();
  const std::vector<std::pair<std::string, std::vector<double>>> want = {
      {"comm.servo_rate_hz", {0, 500, 250}}, {"comm.packet_loss_prob", {0, 1e-5, 1e-4, 1e-3}}};
  for (const auto& [name, levels] : want) {
    const SweepParam* found = nullptr;
    for (const SweepParam& p : sweep.value().params) {
      found = p.name == name ? &p : found;
    }
    ASSERT_NE(found, nullptr) << name;
    EXPECT_EQ(found->levels, levels) << name;
    for (const double level : levels) {
      const auto r = load_imperfections_with<7>(sweep.value().base_profile, name, level);
      ASSERT_TRUE(r.is_ok()) << name << " " << level << ": " << r.reason();
      const imperfect::CommConfig& c = r.value().comm;
      EXPECT_EQ(name == "comm.servo_rate_hz" ? c.servo_rate_hz : c.packet_loss_prob, level) << name;
    }
  }
}

// Sweep params `config.<section>.<key>` replace a scalar of the controller config; the sweep
// passes the param name as is.
TEST(ConfigOverride, HalTimeoutIsSweepable) {
  const fs::path yaml = fs::path(COBOT_REPO_DIR) / "config/default.yaml";
  const auto base = load_config<7>(yaml);
  ASSERT_TRUE(base.is_ok()) << base.reason();
  const auto r = load_config_with<7>(yaml, "config.safety.hal_timeout_ms", 12.0);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  // 12 ms * 1e-3 s/ms, the same product load_config forms.
  EXPECT_EQ(r.value().safety.hal_timeout_s, 12.0 * 1.0e-3);
  EXPECT_EQ(base.value().safety.hal_timeout_s, 5.0 * 1.0e-3);
  // Everything else as loaded without the override.
  EXPECT_EQ(r.value().control_rate_hz, base.value().control_rate_hz);
  EXPECT_EQ(r.value().safety.soft_margin_rad, base.value().safety.soft_margin_rad);
  EXPECT_EQ(r.value().computed_torque.kp, base.value().computed_torque.kp);
  EXPECT_EQ(r.value().urdf_path, base.value().urdf_path);
  // The level is validated like the file value (hal_timeout_ms must be > 0).
  const auto bad = load_config_with<7>(yaml, "config.safety.hal_timeout_ms", -1.0);
  ASSERT_FALSE(bad.is_ok());
  EXPECT_NE(bad.reason().find("hal_timeout_ms"), std::string::npos) << bad.reason();
}

TEST(ConfigOverride, UnknownKeyIsAnErrorNamingIt) {
  const fs::path yaml = fs::path(COBOT_REPO_DIR) / "config/default.yaml";
  const auto r = load_config_with<7>(yaml, "config.safety.no_such_key", 1.0);
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("config.safety.no_such_key"), std::string::npos) << r.reason();
}

}  // namespace
}  // namespace cobot::pipeline

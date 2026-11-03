#include "cobot/pipeline/config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "cobot/common/dof.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/sweep_config.hpp"
#include "cobot/pipeline/waypoint_file.hpp"

// Strict YAML reading: every problem becomes one line "<full.key.path>: <reason>" and all
// lines are returned together, so a config file is fixed in one pass.
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kSecondsPerMillisecond = 1.0e-3;
constexpr int kMaxDelayCycles = 32;
constexpr double kMinEncoderBits = 8.0;
constexpr double kMaxEncoderBits = 32.0;

// Allowed values of a number and the message when a value is outside them.
struct Rule {
  bool (*ok)(double);
  const char* text;
};
const Rule kAny{[](double) { return true; }, ""};
const Rule kPositive{[](double v) { return v > 0.0; }, "must be > 0"};
const Rule kNonNegative{[](double v) { return v >= 0.0; }, "must be >= 0"};
const Rule kAboveOne{[](double v) { return v > 1.0; }, "must be > 1"};
const Rule kFraction{[](double v) { return v >= 0.0 && v < 1.0; }, "must be in [0, 1)"};
const Rule kProbability{[](double v) { return v >= 0.0 && v <= 1.0; }, "must be in [0, 1]"};
const Rule kEncoderBits{
    [](double v) {
      return v == 0.0 || (v == std::floor(v) && v >= kMinEncoderBits && v <= kMaxEncoderBits);
    },
    "must be 0 or an integer in [8, 32]"};

// One YAML mapping. Each key is looked up once; finish() reports keys never asked for. A
// missing or non-map node gives a reader whose lookups do nothing, so errors do not cascade.
class Reader {
 public:
  Reader(const YAML::Node& node, std::string path, std::vector<std::string>& errors)
      : node_(node), path_(std::move(path)), errors_(&errors) {}

  void fail(const std::string& key, const std::string& reason) {
    errors_->push_back(full(key) + ": " + reason);
  }

  void finish() {
    if (!node_.IsMap()) {
      return;
    }
    for (const auto& kv : node_) {
      if (used_.count(kv.first.Scalar()) == 0U) {
        fail(kv.first.Scalar(), "unknown key");
      }
    }
  }

  // The raw child node (empty after reporting "missing"), for lists the helpers do not cover.
  YAML::Node node(const std::string& key) { return get(key); }

  // Reader over `n`, an element of a list of this mapping labelled `label`.
  Reader child(const YAML::Node& n, const std::string& label) {
    if (!n.IsMap()) {
      fail(label, "expected a mapping");
    }
    return {n.IsMap() ? n : YAML::Node(), full(label), *errors_};
  }

  // True when `key` is present and not null, for optional sections; reports nothing. A key
  // present with a null value (`key:` or `key: ~`) counts as absent but is marked as read, so
  // finish() does not call it unknown.
  bool has(const std::string& key) {
    const YAML::Node& map = node_;  // const lookup never inserts the key
    if (!node_.IsMap() || !map[key].IsDefined()) {
      return false;
    }
    used_.insert(key);
    return !map[key].IsNull();
  }

  Reader map(const std::string& key) {
    const YAML::Node n = get(key);
    if (n && !n.IsMap()) {
      fail(key, "expected a mapping");
    }
    return {n && n.IsMap() ? n : YAML::Node(), full(key), *errors_};
  }

  // bool, int, std::uint64_t or non-empty std::string; true when `out` was set.
  template <typename V>
  bool scalar(const std::string& key, V& out) {
    const YAML::Node n = get(key);
    V v{};
    if (!n) {
      return false;
    }
    if (!n.IsScalar() || n.Scalar().empty() || !YAML::convert<V>::decode(n, v)) {
      fail(key, "invalid value '" + YAML::Dump(n) + "'");
      return false;
    }
    out = v;
    return true;
  }

  void number(const std::string& key, const Rule& rule, double& out) {
    const YAML::Node n = get(key);
    double v = 0.0;
    if (n && check(key, n, rule, v)) {
      out = v;
    }
  }

  // Exactly M numbers; with `broadcast`, one scalar sets all M entries.
  template <int M>
  void vector(const std::string& key, const Rule& rule, bool broadcast, VecN<M>& out) {
    const YAML::Node n = get(key);
    double s = 0.0;
    if (!n) {
      return;
    }
    if (broadcast && n.IsScalar()) {
      if (check(key, n, rule, s)) {
        out.setConstant(s);
      }
      return;
    }
    if (!n.IsSequence() || n.size() != static_cast<std::size_t>(M)) {
      fail(key,
           std::string(broadcast ? "expected a scalar or " : "expected ") + std::to_string(M) +
               " values, got " +
               (n.IsSequence() ? std::to_string(n.size()) : "'" + YAML::Dump(n) + "'"));
      return;
    }
    VecN<M> v = VecN<M>::Zero();
    bool ok = true;
    for (int i = 0; i < M; ++i) {
      const std::string label = key + "[" + std::to_string(i) + "]";
      ok = check(label, n[static_cast<std::size_t>(i)], rule, v[i]) && ok;
    }
    if (ok) {
      out = v;
    }
  }

 private:
  std::string full(const std::string& key) const { return path_.empty() ? key : path_ + "." + key; }

  // A finite number from a scalar node that satisfies `rule`; reports under `label`.
  bool check(const std::string& label, const YAML::Node& n, const Rule& rule, double& out) {
    if (!n.IsScalar() || !YAML::convert<double>::decode(n, out) || !std::isfinite(out)) {
      fail(label, "expected a number, got '" + YAML::Dump(n) + "'");
      return false;
    }
    if (!rule.ok(out)) {
      fail(label, std::string(rule.text) + ", got " + n.Scalar());
      return false;
    }
    return true;
  }

  // The child node, or an empty node (after reporting "missing") when the key is absent.
  YAML::Node get(const std::string& key) {
    used_.insert(key);
    if (!node_.IsMap()) {
      return {};
    }
    const YAML::Node& map = node_;  // const lookup never inserts the key
    const YAML::Node n = map[key];
    if (!n.IsDefined() || n.IsNull()) {
      fail(key, "missing");
      return {};
    }
    return n;
  }

  YAML::Node node_;
  std::string path_;
  std::vector<std::string>* errors_;
  std::set<std::string> used_;
};

void delay(Reader& r, const std::string& key, int& out) {
  int v = 0;
  if (!r.scalar(key, v)) {
    return;
  }
  if (v < 0 || v > kMaxDelayCycles) {
    r.fail(key, "must be in [0, 32], got " + std::to_string(v));
  } else {
    out = v;
  }
}

// No change to the loaded document.
std::string keep(YAML::Node& /*root*/) {
  return {};
}

// Loads the file, applies `edit(root)` (a non-empty return is the error), runs
// `read(root, value)` and turns all collected problems into one error.
template <typename T, typename ReadFn, typename EditFn = std::string (*)(YAML::Node&)>
Result<T> load(const fs::path& yaml, ReadFn read, EditFn edit = keep) {
  std::error_code ec;
  if (!fs::is_regular_file(yaml, ec)) {
    return Result<T>::error("cannot open " + yaml.string() + ": no such file");
  }
  std::vector<std::string> errors;
  T value{};
  try {
    // NOLINTNEXTLINE(misc-const-correctness): edit() may change the document
    YAML::Node node = YAML::LoadFile(yaml.string());
    if (!node.IsMap()) {
      return Result<T>::error(yaml.string() + ": expected a mapping at the top level");
    }
    const std::string bad = edit(node);
    if (!bad.empty()) {
      return Result<T>::error(yaml.string() + ": " + bad);
    }
    Reader root(node, "", errors);
    read(root, value);
    root.finish();
  } catch (const YAML::Exception& e) {
    return Result<T>::error(yaml.string() + ": " + e.what());
  }
  if (errors.empty()) {
    return Result<T>::ok(std::move(value));
  }
  std::string msg = yaml.string() + ":";
  for (const std::string& e : errors) {
    msg += "\n" + e;
  }
  return Result<T>::error(msg);
}

// R = Rz(yaw) Ry(pitch) Rx(roll) of rpy = (roll, pitch, yaw) in rad.
common::Mat3 rpy_matrix(const Vec3& rpy) {
  const common::Mat3 r = Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()).toRotationMatrix() *
                         Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()).toRotationMatrix() *
                         Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()).toRotationMatrix();
  return r;
}

// {position_m: [3], rpy_deg: [3]} of `r`; `out` is set only when both are valid.
void read_pose(Reader& r, common::Pose& out) {
  Vec3 p = Vec3::Constant(std::nan(""));
  Vec3 rpy = Vec3::Constant(std::nan(""));
  r.vector<3>("position_m", kAny, false, p);
  r.vector<3>("rpy_deg", kAny, false, rpy);
  if (p.allFinite() && rpy.allFinite()) {
    out = common::Pose{rpy_matrix(rpy * kDegToRad), p};
  }
}

// Reads `key` of `r` into `out` only when present (and not null); otherwise `out` keeps its
// default.
void optional_number(Reader& r, const std::string& key, const Rule& rule, double& out) {
  if (r.has(key)) {
    r.number(key, rule, out);
  }
}

// An angle given in degrees under `key`, stored in rad; absent keeps the default.
void optional_degrees(Reader& r, const std::string& key, double& out_rad) {
  if (r.has(key)) {
    double deg = 0.0;  // stays 0 (not > 0) when the value is rejected
    r.number(key, kPositive, deg);
    out_rad = deg > 0.0 ? deg * kDegToRad : out_rad;
  }
}

// Optional keys of cobot.log (feature pipeline-recorder); a missing one keeps its default.
template <int N>
void read_log_mode(Reader& l, Config<N>& c) {
  if (l.has("mode")) {
    const YAML::Node n = l.node("mode");
    const std::string v = n.IsScalar() ? n.Scalar() : std::string();
    if (v == "off") {
      c.log_mode = LogMode::kOff;
    } else if (v == "recorder") {
      c.log_mode = LogMode::kRecorder;
    } else if (v == "recorder_live") {
      c.log_mode = LogMode::kRecorderLive;
    } else {
      l.fail("mode", "expected off, recorder or recorder_live, got '" + YAML::Dump(n) + "'");
    }
  }
  if (l.has("capacity_s")) {
    l.number("capacity_s", kPositive, c.log_capacity_s);
  }
}

// The optional sections of feature `loop-external`: a missing (or null) section or sub-key keeps
// its default.
template <int N>
void read_external(Reader& m, Config<N>& c) {
  if (m.has("command_source")) {
    const YAML::Node n = m.node("command_source");
    const std::string v = n.IsScalar() ? n.Scalar() : std::string();
    if (v == "config" || v == "external") {
      c.command_source = v == "config" ? CommandSource::kConfig : CommandSource::kExternal;
    } else {
      m.fail("command_source", "expected config or external, got '" + YAML::Dump(n) + "'");
    }
  }
  if (m.has("cartesian_limits")) {
    Reader l = m.map("cartesian_limits");
    optional_number(l, "v_max_m_s", kPositive, c.cartesian_limits.v_max_m_s);
    optional_number(l, "a_max_m_s2", kPositive, c.cartesian_limits.a_max_m_s2);
    optional_number(l, "w_max_rad_s", kPositive, c.cartesian_limits.w_max_rad_s);
    optional_number(l, "alpha_max_rad_s2", kPositive, c.cartesian_limits.alpha_max_rad_s2);
    l.finish();
  }
  if (m.has("stop_time_s")) {
    m.number("stop_time_s", kPositive, c.stop_time_s);
  }
  if (m.has("tcp_offset")) {
    Reader t = m.map("tcp_offset");
    common::Pose& tcp = c.tcp_offset.T_tip_tcp;
    if (t.has("position_m")) {
      Vec3 p = Vec3::Constant(std::nan(""));
      t.vector<3>("position_m", kAny, false, p);
      if (p.allFinite()) {
        tcp.p = p;
      }
    }
    if (t.has("rpy_deg")) {
      Vec3 rpy = Vec3::Constant(std::nan(""));
      t.vector<3>("rpy_deg", kAny, false, rpy);
      if (rpy.allFinite()) {
        tcp.R = rpy_matrix(rpy * kDegToRad);
      }
    }
    t.finish();
  }
  if (m.has("path_validation")) {
    Reader v = m.map("path_validation");
    ValidationLimits& pv = c.path_validation;
    optional_number(v, "reach_pos_m", kPositive, pv.reach_pos_m);
    optional_degrees(v, "reach_rot_deg", pv.reach_rot_rad);
    optional_degrees(v, "joint_margin_deg", pv.joint_margin_rad);
    optional_number(v, "torque_ratio", kPositive, pv.torque_ratio);
    optional_number(v, "max_stretch", kPositive, pv.max_stretch);
    v.finish();
  }
}

// `limits` (feature joint-limits), read after the URDF path and links: the URDF <limit effort>
// (momentary torque M) is the default of both lists and the upper bound of the peak torque. A
// URDF that cannot be read leaves the bound out; building the model reports that file later.
template <int N>
void read_limits(Reader& m, Config<N>& c) {
  VecN<N> effort = VecN<N>::Constant(std::numeric_limits<double>::infinity());
  const Result<model::UrdfChain> chain =
      model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  if (chain && chain.value().joints.size() == static_cast<std::size_t>(N)) {
    for (int j = 0; j < N; ++j) {
      effort[j] = chain.value().joints[static_cast<std::size_t>(j)].tau_max;
    }
  }
  if (!m.has("limits")) {
    c.limits.rated_torque_nm = effort;
    c.limits.peak_torque_nm = effort;
    return;
  }
  Reader l = m.map("limits");
  l.vector<N>("rated_torque_nm", kPositive, false, c.limits.rated_torque_nm);
  l.vector<N>("peak_torque_nm", kPositive, false, c.limits.peak_torque_nm);
  for (int j = 0; j < N; ++j) {
    const double peak = c.limits.peak_torque_nm[j];
    if (std::isfinite(peak) && peak > effort[j]) {
      std::array<char, 96> text{};
      std::snprintf(text.data(),
                    text.size(),
                    "must be <= the URDF effort %.17g N m of joint %d, got %.17g",
                    effort[j],
                    j + 1,
                    peak);
      l.fail("peak_torque_nm[" + std::to_string(j) + "]", text.data());
    }
  }
  l.finish();
}

// `deflection_comp` (feature deflection-comp): optional, every key required inside it. The values
// are checked (> 0, t1 < t2 per joint) only when enabled.
template <int N>
void read_deflection_comp(Reader& m, DeflectionCompConfig<N>& d) {
  if (!m.has("deflection_comp")) {
    return;
  }
  Reader x = m.map("deflection_comp");
  x.scalar("enabled", d.enabled);
  const Rule& rule = d.enabled ? kPositive : kAny;
  x.vector<N>("stiffness_k1_nm_rad", rule, false, d.stiffness_k1_nm_rad);
  x.vector<N>("stiffness_k2_nm_rad", rule, false, d.stiffness_k2_nm_rad);
  x.vector<N>("stiffness_k3_nm_rad", rule, false, d.stiffness_k3_nm_rad);
  x.vector<N>("torque_t1_nm", rule, false, d.torque_t1_nm);
  x.vector<N>("torque_t2_nm", rule, false, d.torque_t2_nm);
  for (int j = 0; d.enabled && j < N; ++j) {
    const double t1 = d.torque_t1_nm[j];
    const double t2 = d.torque_t2_nm[j];
    if (t1 > 0.0 && t2 > 0.0 && !(t2 > t1)) {
      x.fail("torque_t2_nm[" + std::to_string(j) + "]", "must be > torque_t1_nm when enabled");
    }
  }
  x.finish();
}

template <int N>
void read_config(Reader& root, const fs::path& dir, Config<N>& c) {
  Reader m = root.map("cobot");
  std::string text;
  m.scalar("urdf_path", text);
  c.urdf_path = (dir / text).lexically_normal();
  m.scalar("base_link", c.base_link);
  m.scalar("tip_link", c.tip_link);
  m.vector<3>("gravity_b", kAny, false, c.gravity_b);
  m.number("control_rate_hz", kPositive, c.control_rate_hz);
  m.scalar("deterministic", c.deterministic);
  m.vector<N>("home_q_deg", kAny, false, c.home_q_rad);
  c.home_q_rad *= kDegToRad;

  Reader t = m.map("trajectory");
  t.vector<3>("goal_position_m", kAny, false, c.trajectory.goal_position_m);
  t.vector<3>("goal_rpy_deg", kAny, false, c.trajectory.goal_rpy_rad);
  c.trajectory.goal_rpy_rad *= kDegToRad;
  t.number("duration_s", kPositive, c.trajectory.duration_s);
  t.number("hold_s", kNonNegative, c.trajectory.hold_s);

  Reader k = m.map("clik");
  k.vector<6>("kp", kPositive, false, c.clik.kp);
  k.vector<6>("kd", kPositive, false, c.clik.kd);
  k.number("k_null", kPositive, c.clik.k_null);
  k.number("k_damp", kPositive, c.clik.k_damp);
  k.number("lambda_max", kNonNegative, c.clik.lambda_max);
  k.number("cond_threshold", kAboveOne, c.clik.cond_threshold);

  Reader g = m.map("computed_torque");
  g.vector<N>("kp", kPositive, false, c.computed_torque.kp);
  g.vector<N>("kd", kPositive, false, c.computed_torque.kd);

  // Optional: a missing or empty (null) section keeps the disabled defaults.
  if (m.has("friction_compensation")) {
    Reader f = m.map("friction_compensation");
    control::FrictionCompensationConfig<N>& fc = c.friction_compensation;
    f.scalar("enabled", fc.enabled);
    f.vector<N>("coulomb_nm", kNonNegative, true, fc.coulomb_nm);
    f.vector<N>("viscous_nm_s_rad", kNonNegative, true, fc.viscous_nm_s_rad);
    f.number("velocity_eps_rad_s", kPositive, fc.velocity_eps_rad_s);
    f.finish();
  }

  Reader s = m.map("safety");
  double margin_deg = 0.0;
  s.number("soft_margin_deg", kNonNegative, margin_deg);
  c.safety.soft_margin_rad = margin_deg * kDegToRad;
  double timeout_ms = 0.0;
  s.number("hal_timeout_ms", kPositive, timeout_ms);
  c.safety.hal_timeout_s = timeout_ms * kSecondsPerMillisecond;

  Reader l = m.map("log");
  l.scalar("output_dir", text);
  c.log_output_dir = (dir / text).lexically_normal();
  read_log_mode(l, c);
  read_external<N>(m, c);
  read_limits<N>(m, c);
  read_deflection_comp<N>(m, c.deflection_comp);
  for (Reader* r : {&t, &k, &g, &s, &l, &m}) {
    r->finish();
  }
}

// Optional actuator-limit keys (realism-2 design 6.1); absent keys keep the effect off.
template <int N>
void read_actuator_limits(Reader& a, imperfect::ActuatorConfig<N>& act) {
  const auto optional = [&a](const char* key, const Rule& rule, VecN<N>& out) {
    if (a.has(key)) {
      a.vector<N>(key, rule, true, out);
    }
  };
  optional("torque_scale_asym_frac", kFraction, act.torque_scale_asym_frac);
  optional("deadband_nm", kNonNegative, act.deadband_nm);
  optional("torque_slew_nm_s", kNonNegative, act.torque_slew_nm_s);
  optional("knee_speed_rad_s", kNonNegative, act.knee_speed_rad_s);
  optional("zero_torque_speed_rad_s", kNonNegative, act.zero_torque_speed_rad_s);
  for (int j = 0; j < N; ++j) {
    const double knee = act.knee_speed_rad_s[j];
    const double zero = act.zero_torque_speed_rad_s[j];
    if (knee > 0.0 && !(knee < zero)) {
      std::array<char, 96> text{};
      std::snprintf(
          text.data(), text.size(), "must be < zero_torque_speed_rad_s %g, got %g", zero, knee);
      a.fail("knee_speed_rad_s[" + std::to_string(j) + "]", text.data());
    }
  }
  if (a.has("bus")) {
    Reader b = a.map("bus");
    typename imperfect::ActuatorConfig<N>::Bus& bus = act.bus;
    b.number("v_nom_v", kNonNegative, bus.v_nom_v);
    b.number("r_source_ohm", kNonNegative, bus.r_source_ohm);
    b.number("v_min_v", kNonNegative, bus.v_min_v);
    if (bus.v_nom_v > 0.0 && !(bus.v_min_v < bus.v_nom_v)) {
      b.fail("v_min_v", "must be < v_nom_v when the bus is on");
    }
    b.finish();
  }
}

// Transmission group (realism-2 design 6.2): every key a scalar or N values; k3 == 0 is rigid,
// an elastic joint needs k1, k2 > 0 and 0 < t1 < t2.
template <int N>
void read_transmission(Reader& x, imperfect::TransmissionConfig<N>& t) {
  x.vector<N>("stiffness_k1_nm_rad", kNonNegative, true, t.stiffness_k1_nm_rad);
  x.vector<N>("stiffness_k2_nm_rad", kNonNegative, true, t.stiffness_k2_nm_rad);
  x.vector<N>("stiffness_k3_nm_rad", kNonNegative, true, t.stiffness_k3_nm_rad);
  x.vector<N>("torque_t1_nm", kNonNegative, true, t.torque_t1_nm);
  x.vector<N>("torque_t2_nm", kNonNegative, true, t.torque_t2_nm);
  // Optional: an absent key keeps the TransmissionConfig default (1 % damping, no error, no
  // lost motion, ratio 100).
  if (x.has("damping_ratio")) {
    x.vector<N>("damping_ratio", kNonNegative, true, t.damping_ratio);
  }
  if (x.has("error_amp_rad")) {
    x.vector<N>("error_amp_rad", kNonNegative, true, t.error_amp_rad);
  }
  if (x.has("lost_motion_rad")) {
    x.vector<N>("lost_motion_rad", kNonNegative, true, t.lost_motion_rad);
  }
  if (x.has("ratio")) {
    x.vector<N>("ratio", kPositive, true, t.ratio);
  }
  for (int j = 0; j < N; ++j) {
    if (!(t.stiffness_k3_nm_rad[j] > 0.0)) {
      continue;
    }
    const std::string at = "[" + std::to_string(j) + "]";
    if (!(t.stiffness_k1_nm_rad[j] > 0.0)) {
      x.fail("stiffness_k1_nm_rad" + at, "must be > 0 when stiffness_k3_nm_rad > 0");
    }
    if (!(t.stiffness_k2_nm_rad[j] > 0.0)) {
      x.fail("stiffness_k2_nm_rad" + at, "must be > 0 when stiffness_k3_nm_rad > 0");
    }
    if (!(t.torque_t1_nm[j] > 0.0)) {
      x.fail("torque_t1_nm" + at, "must be > 0 when stiffness_k3_nm_rad > 0");
    } else if (!(t.torque_t2_nm[j] > t.torque_t1_nm[j])) {
      x.fail("torque_t2_nm" + at, "must be > torque_t1_nm when stiffness_k3_nm_rad > 0");
    }
  }
}

template <int N>
void read_imperfections(Reader& root, imperfect::ImperfectionConfig<N>& c) {
  Reader m = root.map("imperfections");
  m.scalar("seed", c.seed);
  std::string source;  // required in every group for traceability, not stored

  Reader s = m.map("sensor");
  s.scalar("source", source);
  s.vector<N>("encoder_bits", kEncoderBits, true, c.sensor.encoder_bits);
  s.vector<N>("q_noise_std_rad", kNonNegative, true, c.sensor.q_noise_std_rad);
  s.vector<N>("q_offset_std_rad", kNonNegative, true, c.sensor.q_offset_std_rad);
  s.scalar("qd_from_diff", c.sensor.qd_from_diff);
  s.vector<N>("qd_filter_hz", kPositive, true, c.sensor.qd_filter_hz);
  s.vector<N>("qd_noise_std_rad_s", kNonNegative, true, c.sensor.qd_noise_std_rad_s);
  delay(s, "sensor_delay_cycles", c.sensor.sensor_delay_cycles);
  // Optional (feature transmission): absent reads the link encoder.
  if (s.has("position_source")) {
    std::string v;
    if (s.scalar("position_source", v)) {
      if (v == "link" || v == "motor") {
        c.sensor.position_source =
            v == "motor" ? imperfect::PositionSource::kMotor : imperfect::PositionSource::kLink;
      } else {
        s.fail("position_source", "expected link or motor, got '" + v + "'");
      }
    }
  }

  Reader a = m.map("actuator");
  imperfect::ActuatorConfig<N>& act = c.actuator;
  a.scalar("source", source);
  a.vector<N>("torque_scale_std", kNonNegative, true, act.torque_scale_std);
  a.vector<N>("torque_ripple_frac", kNonNegative, true, act.torque_ripple_frac);
  a.vector<N>("ripple_periods_per_rev", kNonNegative, true, act.ripple_periods_per_rev);
  a.vector<N>("torque_cmd_resolution_nm", kNonNegative, true, act.torque_cmd_resolution_nm);
  a.vector<N>("motor_time_constant_s", kNonNegative, true, act.motor_time_constant_s);
  delay(a, "command_delay_cycles", act.command_delay_cycles);
  read_actuator_limits<N>(a, act);

  Reader f = m.map("friction");
  f.scalar("source", source);
  f.vector<N>("coulomb_nm", kNonNegative, true, c.friction.coulomb_nm);
  f.vector<N>("viscous_nm_s_rad", kNonNegative, true, c.friction.viscous_nm_s_rad);

  Reader d = m.map("model");
  d.scalar("source", source);
  d.vector<N>("link_mass_scale_std", kNonNegative, true, c.model.link_mass_scale_std);
  d.number("com_shift_std_m", kNonNegative, c.model.com_shift_std_m);
  d.number("payload_kg", kNonNegative, c.model.payload_kg);
  // Optional (feature payload-inertia): absent keeps the point mass at the tip origin.
  if (d.has("payload_offset_m")) {
    d.vector<3>("payload_offset_m", kAny, false, c.model.payload_offset_m);
  }
  if (d.has("payload_radius_m")) {
    d.number("payload_radius_m", kNonNegative, c.model.payload_radius_m);
  }

  Reader t = m.map("timing");
  t.scalar("source", source);
  t.number("period_jitter_std_us", kNonNegative, c.timing.period_jitter_std_us);
  // Optional (feature comm-timing): absent keys keep the effect off.
  optional_number(t, "overrun_prob", kProbability, c.timing.overrun_prob);
  if (t.has("overrun_cycles") && t.scalar("overrun_cycles", c.timing.overrun_cycles) &&
      c.timing.overrun_cycles < 0) {
    t.fail("overrun_cycles", "must be >= 0, got " + std::to_string(c.timing.overrun_cycles));
  }
  if (m.has("comm")) {
    Reader k = m.map("comm");
    imperfect::CommConfig& comm = c.comm;
    k.scalar("source", source);
    k.number("servo_rate_hz", kNonNegative, comm.servo_rate_hz);
    k.number("packet_loss_prob", kProbability, comm.packet_loss_prob);
    k.number("latency_burst_prob", kProbability, comm.latency_burst_prob);
    delay(k, "burst_cycles", comm.burst_cycles);
    k.finish();
  }
  if (m.has("transmission")) {
    Reader x = m.map("transmission");
    x.scalar("source", source);
    read_transmission<N>(x, c.transmission);
    x.finish();
  }
  for (Reader* r : {&s, &a, &f, &d, &t, &m}) {
    r->finish();
  }
}

// `key` must be a non-empty list; calls each(label, element) for every element.
template <typename EachFn>
void list(Reader& r, const std::string& key, EachFn each) {
  const YAML::Node seq = r.node(key);
  if (seq && (!seq.IsSequence() || seq.size() == 0U)) {
    r.fail(key, "expected a non-empty list");
  } else if (seq) {
    for (std::size_t i = 0; i < seq.size(); ++i) {
      each(key + "[" + std::to_string(i) + "]", seq[i]);
    }
  }
}

fs::path resolve_base(const fs::path& dir, const fs::path& base) {
  std::error_code ec;
  for (fs::path d = dir; !base.is_absolute() && !d.empty(); d = d.parent_path()) {
    if (fs::is_regular_file(d / base, ec)) {
      return (d / base).lexically_normal();
    }
    if (d == d.parent_path()) {
      break;
    }
  }
  return base.is_absolute() ? base : (dir / base).lexically_normal();
}

void read_sweep(Reader& root, const fs::path& dir, SweepSpec& s) {
  std::string base;
  if (root.scalar("base_profile", base)) {
    s.base_profile = resolve_base(dir, base);
  }
  list(root, "seeds", [&](const std::string& label, const YAML::Node& n) {
    std::uint64_t seed = 0;
    if (!n.IsScalar() || n.Scalar().find('-') != std::string::npos ||
        !YAML::convert<std::uint64_t>::decode(n, seed)) {
      root.fail(label, "expected an unsigned integer, got '" + YAML::Dump(n) + "'");
    }
    s.seeds.push_back(seed);
  });
  list(root, "params", [&](const std::string& label, const YAML::Node& n) {
    Reader p = root.child(n, label);
    SweepParam param;
    p.scalar("name", param.name);
    list(p, "levels", [&](const std::string& level_label, const YAML::Node& l) {
      double v = 0.0;
      if (!l.IsScalar() || !YAML::convert<double>::decode(l, v) || !std::isfinite(v)) {
        p.fail(level_label, "expected a number, got '" + YAML::Dump(l) + "'");
      }
      param.levels.push_back(v);
    });
    p.finish();
    s.params.push_back(param);
  });
}

}  // namespace

Result<SweepSpec> load_sweep(const fs::path& yaml) {
  std::error_code ec;
  const fs::path dir = fs::absolute(yaml, ec).parent_path();
  return load<SweepSpec>(yaml, [&dir](Reader& r, SweepSpec& s) { read_sweep(r, dir, s); });
}

template <int N>
Result<imperfect::ImperfectionConfig<N>> load_imperfections_with(const fs::path& yaml,
                                                                 const std::string& name,
                                                                 double level) {
  const auto edit = [&name, level](YAML::Node& root) -> std::string {
    const std::size_t dot = name.find('.');
    const std::string group = name.substr(0, dot);
    const std::string key = dot == std::string::npos ? "" : name.substr(dot + 1);
    // Iterated, not looked up: a lookup of a missing key yields an invalid node that throws.
    bool found = false;
    for (const auto& g : root["imperfections"]) {
      for (const auto& k :
           g.first.Scalar() == group && g.second.IsMap() ? g.second : YAML::Node()) {
        found = found || k.first.Scalar() == key;
      }
    }
    // Checked first: the reason must not depend on whether the profile sets the key.
    if (name == "model.payload_offset_m") {
      return "sweep param " + name + " is a vector and cannot be swept";
    }
    if (key.empty() || key == "source" || !found) {
      return "unknown sweep param " + name + " (no key imperfections." + name + ")";
    }
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.17g", level);
    root["imperfections"][group][key] = std::string(text.data());
    return {};
  };
  return load<imperfect::ImperfectionConfig<N>>(
      yaml,
      [](Reader& r, imperfect::ImperfectionConfig<N>& c) { read_imperfections<N>(r, c); },
      edit);
}

template <int N>
Result<Config<N>> load_config(const fs::path& yaml) {
  // Non-throwing overload: an empty or bad path is reported by load() as an error.
  std::error_code ec;
  const fs::path dir = fs::absolute(yaml, ec).parent_path();
  return load<Config<N>>(yaml, [&dir](Reader& r, Config<N>& c) { read_config<N>(r, dir, c); });
}

template <int N>
Result<Config<N>> load_config_with(const fs::path& yaml, const std::string& key, double level) {
  const auto edit = [&key, level](YAML::Node& root) -> std::string {
    const std::string prefix = "config.";
    const std::size_t dot = key.find('.', prefix.size());
    const bool shaped = key.compare(0, prefix.size(), prefix) == 0 && dot != std::string::npos;
    const std::string section = shaped ? key.substr(prefix.size(), dot - prefix.size()) : "";
    const std::string name = shaped ? key.substr(dot + 1) : "";
    // Iterated, not looked up: a lookup of a missing key yields an invalid node that throws.
    bool scalar = false;
    for (const auto& s : root["cobot"]) {
      for (const auto& k :
           s.first.Scalar() == section && s.second.IsMap() ? s.second : YAML::Node()) {
        scalar = scalar || (k.first.Scalar() == name && k.second.IsScalar());
      }
    }
    if (!scalar) {
      return "config param " + key + " is not sweepable (no scalar cobot." + section + "." + name +
             ")";
    }
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.17g", level);
    root["cobot"][section][name] = std::string(text.data());
    return {};
  };
  std::error_code ec;
  const fs::path dir = fs::absolute(yaml, ec).parent_path();
  return load<Config<N>>(
      yaml, [&dir](Reader& r, Config<N>& c) { read_config<N>(r, dir, c); }, edit);
}

template <int N>
model::Model<N> controller_model(const model::Model<N>& urdf_model, const Config<N>& config) {
  model::Model<N> out = urdf_model;
  for (int j = 0; j < N; ++j) {
    double& tau_max = out.joints[static_cast<std::size_t>(j)].tau_max;
    tau_max = std::min(config.limits.peak_torque_nm[j], tau_max);
  }
  return out;
}

// Declared in waypoint_file.hpp; parsed here with the strict reader of the config files.
Result<std::vector<planning::Waypoint>> load_waypoints(const fs::path& yaml,
                                                       const std::string& base_link,
                                                       const TcpOffset& tcp) {
  const common::Mat3 r_tcp_tip = tcp.T_tip_tcp.R.transpose();
  const common::Pose T_tcp_tip{r_tcp_tip, -(r_tcp_tip * tcp.T_tip_tcp.p)};
  const auto read = [&](Reader& root, std::vector<planning::Waypoint>& out) {
    std::string frame;
    if (root.scalar("frame", frame) && frame != base_link) {
      root.fail("frame", "expected " + base_link + ", got " + frame);
    }
    list(root, "waypoints", [&](const std::string& label, const YAML::Node& n) {
      Reader w = root.child(n, label);
      planning::Waypoint wp;
      common::Pose T_b_tcp;
      read_pose(w, T_b_tcp);
      wp.T_b_goal = common::compose(T_b_tcp, T_tcp_tip);
      if (w.has("duration_s")) {
        w.number("duration_s", kNonNegative, wp.duration_s);
      }
      if (w.has("stop")) {
        w.scalar("stop", wp.stop);
      }
      w.finish();
      out.push_back(wp);
    });
  };
  return load<std::vector<planning::Waypoint>>(yaml, read);
}

template <int N>
Result<imperfect::ImperfectionConfig<N>> load_imperfections(const fs::path& yaml) {
  return load<imperfect::ImperfectionConfig<N>>(
      yaml, [](Reader& r, imperfect::ImperfectionConfig<N>& c) { read_imperfections<N>(r, c); });
}

// NOLINTBEGIN(bugprone-macro-parentheses): N is a template argument
#define COBOT_INSTANTIATE(N)                                                                   \
  template Result<Config<N>> load_config<N>(const fs::path&);                                  \
  template Result<Config<N>> load_config_with<N>(const fs::path&, const std::string&, double); \
  template model::Model<N> controller_model<N>(const model::Model<N>&, const Config<N>&);      \
  template Result<imperfect::ImperfectionConfig<N>> load_imperfections<N>(const fs::path&);    \
  template Result<imperfect::ImperfectionConfig<N>> load_imperfections_with<N>(                \
      const fs::path&, const std::string&, double);
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE
// NOLINTEND(bugprone-macro-parentheses)

}  // namespace cobot::pipeline

#include "cobot/studio/param_catalog.hpp"

#include <cmath>
#include <string>
#include <utility>

#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"

namespace cobot::studio {
namespace {

constexpr int kN = model::kArmDof;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kMsPerS = 1e3;
constexpr double kOmegaMin = 10.0;      // rad/s, design 5.3
constexpr double kOmegaMax = 200.0;     // rad/s
constexpr double kHalTimeoutMin = 1.0;  // ms
constexpr double kHalTimeoutMax = 50.0;
using Cfg = pipeline::Config<kN>;
using Imp = imperfect::ImperfectionConfig<kN>;
using Value = decltype(pipeline::KeyOverride::value);
using Getter = Value (*)(const Cfg&, const Imp&);

struct Entry {
  ParamInfo info;
  Getter get;
};

template <int M>
Value list(const common::VecN<M>& v) {
  return std::vector<double>(v.data(), v.data() + M);
}

ParamInfo info(const char* key,
               const char* group,
               const char* unit,
               ParamKind kind,
               NoiseSource noise,
               bool sweepable = true) {
  ParamInfo p;
  p.key = key;
  p.group = group;
  p.unit = unit;
  p.kind = kind;
  p.noise = noise;
  p.sweepable = sweepable && (kind == ParamKind::kNumber || kind == ParamKind::kPerJoint);
  return p;
}

ParamInfo limited(ParamInfo p, double min, double max) {
  p.min = min;
  p.max = max;
  return p;
}

ParamInfo choice(ParamInfo p, std::vector<std::string> choices) {
  p.choices = std::move(choices);
  return p;
}

ParamInfo bench_only(ParamInfo p) {
  p.gazebo_supported = false;
  return p;
}

// Getters of the loaded files: I for an imperfection field, C for a controller one.
// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
#define COBOT_PJ(field) [](const Cfg&, const Imp& i) -> Value { return list(i.field); }
#define COBOT_NUM(field) \
  [](const Cfg&, const Imp& i) -> Value { return static_cast<double>(i.field); }
#define COBOT_BOOL(field) [](const Cfg&, const Imp& i) -> Value { return i.field; }
// NOLINTEND(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)

std::vector<Entry> make_catalog() {
  using K = ParamKind;
  using S = NoiseSource;
  const S sen = S::kSensor;
  const S act = S::kActuator;
  const S fri = S::kFriction;
  const S none = S::kNone;
  return {
      // Controller (config.<path under cobot:>).
      {limited(info(pipeline::kOmegaKey, "Controller", "rad/s", K::kNumber, none),
               kOmegaMin,
               kOmegaMax),
       [](const Cfg& c, const Imp&) -> Value { return std::sqrt(c.computed_torque.kp(0)); }},
      // kp and kd are per-joint lists in the files, which bench::sweep cannot sweep as one level
      // (it needs a scalar cobot.<section>.<key>); omega is the sweepable gain.
      {info("config.computed_torque.kp", "Controller", "1/s^2", K::kPerJoint, none, false),
       [](const Cfg& c, const Imp&) -> Value { return list(c.computed_torque.kp); }},
      {info("config.computed_torque.kd", "Controller", "1/s", K::kPerJoint, none, false),
       [](const Cfg& c, const Imp&) -> Value { return list(c.computed_torque.kd); }},
      {limited(info("config.safety.hal_timeout_ms", "Controller", "ms", K::kNumber, none),
               kHalTimeoutMin,
               kHalTimeoutMax),
       [](const Cfg& c, const Imp&) -> Value { return c.safety.hal_timeout_s * kMsPerS; }},
      {info("config.safety.soft_margin_deg", "Controller", "deg", K::kNumber, none),
       [](const Cfg& c, const Imp&) -> Value { return c.safety.soft_margin_rad / kDegToRad; }},
      {info("config.friction_compensation.enabled", "Controller", "", K::kBool, none),
       [](const Cfg& c, const Imp&) -> Value { return c.friction_compensation.enabled; }},
      {info("config.deflection_comp.enabled", "Controller", "", K::kBool, none),
       [](const Cfg& c, const Imp&) -> Value { return c.deflection_comp.enabled; }},
      // Sensor.
      {info("sensor.encoder_bits", "Sensor", "bit", K::kPerJoint, sen),
       COBOT_PJ(sensor.encoder_bits)},
      {info("sensor.q_noise_std_rad", "Sensor", "rad", K::kPerJoint, sen),
       COBOT_PJ(sensor.q_noise_std_rad)},
      {info("sensor.q_offset_std_rad", "Sensor", "rad", K::kPerJoint, sen),
       COBOT_PJ(sensor.q_offset_std_rad)},
      {info("sensor.qd_from_diff", "Sensor", "", K::kBool, sen), COBOT_BOOL(sensor.qd_from_diff)},
      {info("sensor.qd_filter_hz", "Sensor", "Hz", K::kPerJoint, sen),
       COBOT_PJ(sensor.qd_filter_hz)},
      {info("sensor.qd_noise_std_rad_s", "Sensor", "rad/s", K::kPerJoint, sen),
       COBOT_PJ(sensor.qd_noise_std_rad_s)},
      {info("sensor.sensor_delay_cycles", "Sensor", "cycles", K::kNumber, sen),
       COBOT_NUM(sensor.sensor_delay_cycles)},
      {choice(info("sensor.position_source", "Sensor", "", K::kChoice, sen), {"link", "motor"}),
       [](const Cfg&, const Imp& i) -> Value {
         return std::string(i.sensor.position_source == imperfect::PositionSource::kMotor ? "motor"
                                                                                          : "link");
       }},
      // Actuator.
      {info("actuator.torque_scale_std", "Actuator", "", K::kPerJoint, act),
       COBOT_PJ(actuator.torque_scale_std)},
      {info("actuator.torque_ripple_frac", "Actuator", "", K::kPerJoint, act),
       COBOT_PJ(actuator.torque_ripple_frac)},
      {info("actuator.ripple_periods_per_rev", "Actuator", "1/rev", K::kPerJoint, act),
       COBOT_PJ(actuator.ripple_periods_per_rev)},
      {info("actuator.torque_cmd_resolution_nm", "Actuator", "N m", K::kPerJoint, act),
       COBOT_PJ(actuator.torque_cmd_resolution_nm)},
      {info("actuator.motor_time_constant_s", "Actuator", "s", K::kPerJoint, act),
       COBOT_PJ(actuator.motor_time_constant_s)},
      {info("actuator.command_delay_cycles", "Actuator", "cycles", K::kNumber, act),
       COBOT_NUM(actuator.command_delay_cycles)},
      {info("actuator.torque_scale_asym_frac", "Actuator", "", K::kPerJoint, act),
       COBOT_PJ(actuator.torque_scale_asym_frac)},
      {info("actuator.deadband_nm", "Actuator", "N m", K::kPerJoint, act),
       COBOT_PJ(actuator.deadband_nm)},
      {info("actuator.torque_slew_nm_s", "Actuator", "N m/s", K::kPerJoint, act),
       COBOT_PJ(actuator.torque_slew_nm_s)},
      {info("actuator.knee_speed_rad_s", "Actuator", "rad/s", K::kPerJoint, act),
       COBOT_PJ(actuator.knee_speed_rad_s)},
      {info("actuator.zero_torque_speed_rad_s", "Actuator", "rad/s", K::kPerJoint, act),
       COBOT_PJ(actuator.zero_torque_speed_rad_s)},
      {info("actuator.bus.v_nom_v", "Actuator", "V", K::kNumber, act),
       COBOT_NUM(actuator.bus.v_nom_v)},
      {info("actuator.bus.r_source_ohm", "Actuator", "ohm", K::kNumber, act),
       COBOT_NUM(actuator.bus.r_source_ohm)},
      {info("actuator.bus.v_min_v", "Actuator", "V", K::kNumber, act),
       COBOT_NUM(actuator.bus.v_min_v)},
      // Friction.
      {info("friction.coulomb_nm", "Friction", "N m", K::kPerJoint, fri),
       COBOT_PJ(friction.coulomb_nm)},
      {info("friction.viscous_nm_s_rad", "Friction", "N m s/rad", K::kPerJoint, fri),
       COBOT_PJ(friction.viscous_nm_s_rad)},
      // Model.
      {info("model.link_mass_scale_std", "Model", "", K::kPerJoint, none),
       COBOT_PJ(model.link_mass_scale_std)},
      {info("model.com_shift_std_m", "Model", "m", K::kNumber, none),
       COBOT_NUM(model.com_shift_std_m)},
      {info("model.payload_kg", "Model", "kg", K::kNumber, none), COBOT_NUM(model.payload_kg)},
      // Three values (x, y, z in link_7), not one per joint; never swept.
      {info("model.payload_offset_m", "Model", "m (x, y, z)", K::kPerJoint, none, false),
       COBOT_PJ(model.payload_offset_m)},
      {info("model.payload_radius_m", "Model", "m", K::kNumber, none),
       COBOT_NUM(model.payload_radius_m)},
      // Timing and comm (bench only).
      {bench_only(info("timing.period_jitter_std_us", "Timing", "us", K::kNumber, none)),
       COBOT_NUM(timing.period_jitter_std_us)},
      {bench_only(info("timing.overrun_prob", "Timing", "", K::kNumber, none)),
       COBOT_NUM(timing.overrun_prob)},
      {bench_only(info("timing.overrun_cycles", "Timing", "cycles", K::kNumber, none)),
       COBOT_NUM(timing.overrun_cycles)},
      {bench_only(info("comm.servo_rate_hz", "Comm", "Hz", K::kNumber, none)),
       COBOT_NUM(comm.servo_rate_hz)},
      {bench_only(info("comm.packet_loss_prob", "Comm", "", K::kNumber, none)),
       COBOT_NUM(comm.packet_loss_prob)},
      {bench_only(info("comm.latency_burst_prob", "Comm", "", K::kNumber, none)),
       COBOT_NUM(comm.latency_burst_prob)},
      {bench_only(info("comm.burst_cycles", "Comm", "cycles", K::kNumber, none)),
       COBOT_NUM(comm.burst_cycles)},
      // Transmission.
      {info("transmission.stiffness_k1_nm_rad", "Transmission", "N m/rad", K::kPerJoint, none),
       COBOT_PJ(transmission.stiffness_k1_nm_rad)},
      {info("transmission.stiffness_k2_nm_rad", "Transmission", "N m/rad", K::kPerJoint, none),
       COBOT_PJ(transmission.stiffness_k2_nm_rad)},
      {info("transmission.stiffness_k3_nm_rad", "Transmission", "N m/rad", K::kPerJoint, none),
       COBOT_PJ(transmission.stiffness_k3_nm_rad)},
      {info("transmission.torque_t1_nm", "Transmission", "N m", K::kPerJoint, none),
       COBOT_PJ(transmission.torque_t1_nm)},
      {info("transmission.torque_t2_nm", "Transmission", "N m", K::kPerJoint, none),
       COBOT_PJ(transmission.torque_t2_nm)},
      {info("transmission.damping_ratio", "Transmission", "", K::kPerJoint, none),
       COBOT_PJ(transmission.damping_ratio)},
      {info("transmission.error_amp_rad", "Transmission", "rad", K::kPerJoint, none),
       COBOT_PJ(transmission.error_amp_rad)},
      {info("transmission.lost_motion_rad", "Transmission", "rad", K::kPerJoint, none),
       COBOT_PJ(transmission.lost_motion_rad)},
      {info("transmission.ratio", "Transmission", "", K::kPerJoint, none),
       COBOT_PJ(transmission.ratio)},
  };
}

#undef COBOT_PJ
#undef COBOT_NUM
#undef COBOT_BOOL

// What cobot_controller rejects (payload) or ignores with a warning (friction, set in the physics
// at launch; transmission; sensor.position_source) on the Gazebo path (design 6.5, on_configure),
// besides the timing and comm keys marked bench_only.
bool gazebo_ignores(const std::string& key) {
  const auto starts = [&key](const char* prefix) { return key.rfind(prefix, 0) == 0; };
  return key == "model.payload_kg" || key == "model.payload_offset_m" ||
         key == "model.payload_radius_m" || key == "sensor.position_source" ||
         starts("friction.") || starts("transmission.");
}

const std::vector<Entry>& entries() {
  static const std::vector<Entry> entries_list = [] {
    std::vector<Entry> out = make_catalog();
    for (Entry& e : out) {
      e.info.gazebo_supported = e.info.gazebo_supported && !gazebo_ignores(e.info.key);
    }
    return out;
  }();
  return entries_list;
}

}  // namespace

const std::vector<ParamInfo>& param_catalog() {
  static const std::vector<ParamInfo> catalog = [] {
    std::vector<ParamInfo> out;
    for (const Entry& e : entries()) {
      out.push_back(e.info);
    }
    return out;
  }();
  return catalog;
}

const ParamInfo* find_param(std::string_view key) {
  for (const ParamInfo& p : param_catalog()) {
    if (p.key == key) {
      return &p;
    }
  }
  return nullptr;
}

common::Result<pipeline::KeyOverride> base_value(const BasePair& base, const std::string& key) {
  using Out = common::Result<pipeline::KeyOverride>;
  for (const Entry& e : entries()) {
    if (e.info.key != key) {
      continue;
    }
    Cfg config;
    Imp imperfections;
    if (key.rfind("config.", 0) == 0) {
      common::Result<Cfg> c = pipeline::load_config<kN>(base.config_yaml);
      if (!c) {
        return Out::error(c.reason());
      }
      config = std::move(c.value());
    } else {
      common::Result<Imp> i = pipeline::load_imperfections<kN>(base.imperfections_yaml);
      if (!i) {
        return Out::error(i.reason());
      }
      imperfections = std::move(i.value());
    }
    return Out::ok(pipeline::KeyOverride{key, e.get(config, imperfections)});
  }
  return Out::error("unknown parameter " + key);
}

std::map<std::string, pipeline::KeyOverride> base_values(const BasePair& base) {
  const common::Result<Cfg> c = pipeline::load_config<kN>(base.config_yaml);
  const common::Result<Imp> i = pipeline::load_imperfections<kN>(base.imperfections_yaml);
  const Cfg config = c ? c.value() : Cfg{};
  const Imp imperfections = i ? i.value() : Imp{};
  std::map<std::string, pipeline::KeyOverride> out;
  for (const Entry& e : entries()) {
    if (e.info.key.rfind("config.", 0) == 0 ? c.is_ok() : i.is_ok()) {
      out[e.info.key] = pipeline::KeyOverride{e.info.key, e.get(config, imperfections)};
    }
  }
  return out;
}

}  // namespace cobot::studio

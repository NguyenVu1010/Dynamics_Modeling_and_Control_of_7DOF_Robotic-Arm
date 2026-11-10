#include "cobot/studio/param_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
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

// Presentation of each key (studio config UI design 3.2): the display unit is chosen so that the
// values of the six profiles under config/ read between 0.1 and 1000 (counts, fractions and the
// stiffnesses excepted); "" exactly when the YAML unit is "".
struct Presentation {
  const char* key;
  const char* label;
  const char* description;
  const char* display_unit;
  double display_scale;
};

constexpr double kMilli = 1e3;   // YAML unit -> milli-unit
constexpr double kMicro = 1e6;   // YAML unit -> micro-unit
constexpr double kPerMs = 1e-3;  // per second -> per millisecond

// clang-format off
constexpr Presentation kPresentation[] = {
    {"config.computed_torque.omega", "Controller bandwidth",
     "Natural frequency of the error dynamics; kp = omega^2, kd = 2 omega", "rad/s", 1.0},
    {"config.computed_torque.kp", "Position gain kp", "Proportional gain, per joint", "1/s²", 1.0},
    {"config.computed_torque.kd", "Velocity gain kd", "Derivative gain, per joint", "1/s", 1.0},
    {"config.safety.hal_timeout_ms", "State timeout",
     "Hold when no fresh state arrives for this long", "ms", 1.0},
    {"config.safety.soft_margin_deg", "Joint limit margin",
     "Hold when a joint gets this close to a limit", "deg", 1.0},
    {"config.friction_compensation.enabled", "Friction compensation",
     "Feed forward the modelled friction torque", "", 1.0},
    {"config.deflection_comp.enabled", "Deflection compensation",
     "Correct the gearbox deflection under load", "", 1.0},
    {"sensor.encoder_bits", "Encoder resolution",
     "Encoder bits per turn, per joint; 0 is no quantisation", "bit", 1.0},
    {"sensor.q_noise_std_rad", "Position noise q",
     "Encoder noise standard deviation, per joint", "µrad", kMicro},
    {"sensor.q_offset_std_rad", "Position offset q",
     "Standard deviation of the calibration offset, per joint", "mrad", kMilli},
    {"sensor.qd_from_diff", "Velocity from position",
     "Velocity by differencing the measured position", "", 1.0},
    {"sensor.qd_filter_hz", "Velocity filter cutoff",
     "Low-pass cutoff of the velocity estimate, per joint", "Hz", 1.0},
    {"sensor.qd_noise_std_rad_s", "Velocity noise qd",
     "Velocity noise standard deviation, per joint", "mrad/s", kMilli},
    {"sensor.sensor_delay_cycles", "Sensor delay",
     "Control cycles before a reading arrives", "cycles", 1.0},
    {"sensor.position_source", "Position source",
     "Encoder the position loop closes on: link or motor side", "", 1.0},
    {"actuator.torque_scale_std", "Torque gain error",
     "Standard deviation of the torque gain, as a fraction", "", 1.0},
    {"actuator.torque_ripple_frac", "Torque ripple",
     "Ripple amplitude as a fraction of the torque", "", 1.0},
    {"actuator.ripple_periods_per_rev", "Ripple periods",
     "Ripple periods per motor revolution", "1/rev", 1.0},
    {"actuator.torque_cmd_resolution_nm", "Torque resolution",
     "Quantisation step of the torque command", "mN m", kMilli},
    {"actuator.motor_time_constant_s", "Motor time constant",
     "First-order lag of the torque response", "ms", kMilli},
    {"actuator.command_delay_cycles", "Command delay",
     "Control cycles before a command takes effect", "cycles", 1.0},
    {"actuator.torque_scale_asym_frac", "Torque asymmetry",
     "Gain loss in the negative direction, as a fraction", "", 1.0},
    {"actuator.deadband_nm", "Torque deadband",
     "Commands below this give no torque", "mN m", kMilli},
    {"actuator.torque_slew_nm_s", "Torque slew limit",
     "Fastest change of the motor torque", "N m/ms", kPerMs},
    {"actuator.knee_speed_rad_s", "Knee speed",
     "Speed where the torque limit starts to fall", "rad/s", 1.0},
    {"actuator.zero_torque_speed_rad_s", "Zero-torque speed",
     "Speed where the motor gives no more torque", "rad/s", 1.0},
    {"actuator.bus.v_nom_v", "Bus voltage",
     "Nominal supply voltage; 0 turns the sag off", "V", 1.0},
    {"actuator.bus.r_source_ohm", "Bus resistance",
     "Source resistance of the shared supply", "mΩ", kMilli},
    {"actuator.bus.v_min_v", "Bus minimum voltage",
     "Lowest voltage the drives still run at", "V", 1.0},
    {"friction.coulomb_nm", "Coulomb friction",
     "Constant friction torque, per joint", "N m", 1.0},
    {"friction.viscous_nm_s_rad", "Viscous friction",
     "Friction torque per unit speed, per joint", "N m s/rad", 1.0},
    {"model.link_mass_scale_std", "Link mass error",
     "Standard deviation of each link mass, as a fraction", "", 1.0},
    {"model.com_shift_std_m", "Centre of mass error",
     "Standard deviation of the centre-of-mass shift", "mm", kMilli},
    {"model.payload_kg", "Payload mass",
     "Unmodelled mass held at the flange", "kg", 1.0},
    {"model.payload_offset_m", "Payload offset",
     "Payload centre in the link 7 frame, x, y, z", "mm", kMilli},
    {"model.payload_radius_m", "Payload radius",
     "Radius of the payload as a solid sphere", "mm", kMilli},
    {"timing.period_jitter_std_us", "Period jitter",
     "Standard deviation of the control period", "µs", 1.0},
    {"timing.overrun_prob", "Overrun probability",
     "Chance that a cycle overruns", "", 1.0},
    {"timing.overrun_cycles", "Overrun length",
     "Extra periods an overrun lasts", "cycles", 1.0},
    {"comm.servo_rate_hz", "Servo rate",
     "Rate of the servo packets; 0 is every control cycle", "Hz", 1.0},
    {"comm.packet_loss_prob", "Packet loss",
     "Chance that a servo packet is lost, each direction", "", 1.0},
    {"comm.latency_burst_prob", "Latency burst",
     "Chance per cycle that a latency burst starts", "", 1.0},
    {"comm.burst_cycles", "Burst length",
     "Reads that repeat the last sample in a burst", "cycles", 1.0},
    {"transmission.stiffness_k1_nm_rad", "Stiffness K1",
     "Gearbox stiffness below torque T1, per joint", "N m/rad", 1.0},
    {"transmission.stiffness_k2_nm_rad", "Stiffness K2",
     "Gearbox stiffness between torques T1 and T2, per joint", "N m/rad", 1.0},
    {"transmission.stiffness_k3_nm_rad", "Stiffness K3",
     "Gearbox stiffness above torque T2, per joint", "N m/rad", 1.0},
    {"transmission.torque_t1_nm", "Breakpoint T1",
     "Torque where stiffness K2 starts, per joint", "N m", 1.0},
    {"transmission.torque_t2_nm", "Breakpoint T2",
     "Torque where stiffness K3 starts, per joint", "N m", 1.0},
    {"transmission.damping_ratio", "Gearbox damping",
     "Gearbox damping as a fraction of critical", "", 1.0},
    {"transmission.error_amp_rad", "Transmission error",
     "Amplitude of the gearbox angle error", "mrad", kMilli},
    {"transmission.lost_motion_rad", "Lost motion",
     "Backlash width of the gearbox", "mrad", kMilli},
    {"transmission.ratio", "Gear ratio",
     "Motor turns per joint turn", "", 1.0},
};
// clang-format on

void present(ParamInfo& p) {
  for (const Presentation& row : kPresentation) {
    if (p.key == row.key) {
      p.label = row.label;
      p.description = row.description;
      p.display_unit = row.display_unit;
      p.display_scale = row.display_scale;
      break;
    }
  }
  p.xyz = p.key == "model.payload_offset_m";
}

const std::vector<Entry>& entries() {
  static const std::vector<Entry> entries_list = [] {
    std::vector<Entry> out = make_catalog();
    for (Entry& e : out) {
      e.info.gazebo_supported = e.info.gazebo_supported && !gazebo_ignores(e.info.key);
      present(e.info);
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

std::string format_display(double display_value) {
  if (!std::isfinite(display_value)) {
    return std::to_string(display_value);
  }
  constexpr int kSignificant = 3;
  constexpr int kMaxDecimals = 15;
  constexpr double kInteger = 1000.0;  // from this magnitude on, rounded to an integer
  const double magnitude = std::abs(display_value);
  int decimals = 0;
  if (magnitude > 0.0 && magnitude < kInteger) {
    const int exponent = static_cast<int>(std::floor(std::log10(magnitude)));
    decimals = std::min(std::max(kSignificant - 1 - exponent, 0), kMaxDecimals);
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(decimals) << display_value;
  std::string text = out.str();
  if (text.find('.') != std::string::npos) {
    text.erase(text.find_last_not_of('0') + 1);
    if (text.back() == '.') {
      text.pop_back();
    }
  }
  if (text == "-0") {
    text = "0";
  }
  return text;
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

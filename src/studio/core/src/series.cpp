#include "cobot/studio/series.hpp"

#include "cobot/common/lie.hpp"

namespace cobot::studio {
namespace {

using Record = pipeline::LogRecord<bench::kN>;

double joint_value(const Record& r, Channel c, int j, NoiseSource source) {
  switch (c) {
    case Channel::kQRef:
      return r.q_ref(j);
    case Channel::kQ:
      return r.q(j);
    case Channel::kQd:
      return r.qd(j);
    case Channel::kQdd:
      return r.qdd_ref(j);
    case Channel::kQTrue:
      return r.q_true(j);
    case Channel::kNoise:
      return source == NoiseSource::kSensor ? r.q(j) - r.q_true(j) : r.tau_friction(j);
    case Channel::kError:
      return r.q_ref(j) - r.q(j);
    case Channel::kTau:
      return r.tau(j);
    case Channel::kTauMotor:
      return r.tau_motor(j);
    case Channel::kTauFriction:
      return r.tau_friction(j);
    case Channel::kTaskPosErr:
      return (r.T_b_d.p - r.T_b_e.p).norm();
    case Channel::kTaskRotErr:
      return common::so3_log(r.T_b_d.R * r.T_b_e.R.transpose()).norm();
  }
  return 0.0;
}

}  // namespace

Series extract(const bench::RunData& run, Channel channel, int joint, NoiseSource source) {
  Series s;
  const bool task = channel == Channel::kTaskPosErr || channel == Channel::kTaskRotErr;
  if (!task && (joint < 0 || joint >= bench::kN)) {
    s.unavailable = "no joint " + std::to_string(joint);
    return s;
  }
  if (channel == Channel::kNoise && source == NoiseSource::kNone) {
    s.unavailable = "no noise line for model, timing or comm parameters (design 5.1)";
    return s;
  }
  const std::vector<Record>& rows = run.records;
  const bool shifted = channel == Channel::kNoise && source == NoiseSource::kActuator;
  const std::size_t n = shifted && !rows.empty() ? rows.size() - 1 : rows.size();
  s.t_s.reserve(n);
  s.y.reserve(n);
  for (std::size_t k = 0; k < n; ++k) {
    s.t_s.push_back(static_cast<double>(rows[k].t.ns) * 1e-9);
    s.y.push_back(shifted ? rows[k + 1].tau_motor(joint) - rows[k].tau(joint)
                          : joint_value(rows[k], channel, task ? 0 : joint, source));
  }
  return s;
}

NoiseSource noise_source_of(const StudioConfig& config) {
  for (const pipeline::KeyOverride& o : config.overrides) {
    const ParamInfo* p = find_param(o.key);
    if (p != nullptr && p->noise != NoiseSource::kNone) {
      return p->noise;
    }
  }
  return NoiseSource::kNone;
}

}  // namespace cobot::studio

#pragma once

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/planning/clik.hpp"

// Shared set-up of the controller-registry tests (ADR-023, brief controller-registry): the arm of a
// controller config with its URDF model (the plant's) and its controller model (peak torque P),
// and random (state, reference) pairs with fixed seeds.
namespace cobot::control::registry_test {

constexpr int kDof = 7;
using Vec7 = common::VecN<kDof>;
using State7 = hal::JointState<kDof>;
using Ref7 = planning::JointReference<kDof>;

constexpr double kFiveDeg = 5.0 * pipeline::kDeg;

struct ArmSetup {
  pipeline::Config<kDof> config;
  model::Arm7 urdf;     // plant model (torque cap M)
  model::Arm7 nominal;  // pipeline::controller_model (tau_max = peak torque P)
};

// `yaml` relative to the repository (default: config/default.yaml).
inline std::optional<ArmSetup> load_arm_setup(const char* yaml = "config/default.yaml") {
  const auto cfg = pipeline::load_config<kDof>(std::filesystem::path(COBOT_REPO_DIR) / yaml);
  if (!cfg) {
    ADD_FAILURE() << "load_config: " << cfg.reason();
    return std::nullopt;
  }
  const pipeline::Config<kDof>& c = cfg.value();
  const auto chain = model::parse_urdf_chain(c.urdf_path, c.base_link, c.tip_link);
  if (!chain) {
    ADD_FAILURE() << "parse_urdf_chain: " << chain.reason();
    return std::nullopt;
  }
  const auto arm = model::make_model<kDof>(chain.value(), c.gravity_b);
  if (!arm) {
    ADD_FAILURE() << "make_model: " << arm.reason();
    return std::nullopt;
  }
  return ArmSetup{c, arm.value(), pipeline::controller_model<kDof>(arm.value(), c)};
}

// q uniform in [q_min + 5 deg, q_max - 5 deg] per joint.
inline Vec7 random_q(const model::Arm7& m, std::mt19937_64& rng) {
  Vec7 q;
  for (int i = 0; i < kDof; ++i) {
    const model::JointParams& j = m.joints[static_cast<std::size_t>(i)];
    std::uniform_real_distribution<double> d(j.q_min + kFiveDeg, j.q_max - kFiveDeg);
    q(i) = d(rng);
  }
  return q;
}

// Every element uniform in [-a, a].
inline Vec7 random_vec(double a, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> d(-a, a);
  Vec7 v;
  for (int i = 0; i < kDof; ++i) {
    v(i) = d(rng);
  }
  return v;
}

// State: q within limits minus 5 deg, qd in [-1, 1]. Reference near it, as in tracking:
// q_ref = q + [-1e-3, 1e-3] rad, qd_ref = qd + [-0.02, 0.02] rad/s, qdd_ref in [-2, 2] rad/s^2,
// so most joints stay below tau_max at kp = 6400 (6400 * 1e-3 + 160 * 0.02 + 2 = 11.6 rad/s^2).
inline void random_pair(const model::Arm7& m, std::mt19937_64& rng, State7& s, Ref7& ref) {
  s.q = random_q(m, rng);
  s.qd = random_vec(1.0, rng);
  ref.q = s.q + random_vec(1e-3, rng);
  ref.qd = s.qd + random_vec(0.02, rng);
  ref.qdd = random_vec(2.0, rng);
}

inline Vec7 tau_max_of(const model::Arm7& m) {
  Vec7 v;
  for (int i = 0; i < kDof; ++i) {
    v(i) = m.joints[static_cast<std::size_t>(i)].tau_max;
  }
  return v;
}

inline std::uint64_t bits_of(double v) {
  std::uint64_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  return u;
}

// Same object representation element by element (what memcmp of the Eigen data compares; NaN
// payloads and the sign of zero count).
template <typename T>
bool bits_equal(const T& a, const T& b) {
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    if (bits_of(a.data()[i]) != bits_of(b.data()[i])) {
      return false;
    }
  }
  return true;
}

inline bool bits_equal(double a, double b) {
  return bits_of(a) == bits_of(b);
}

// Element-wise clamp to [-tau_max, tau_max], as detail::clamp_or_zero does for a finite raw torque.
inline Vec7 clamped(const model::Arm7& m, const Vec7& raw) {
  const Vec7 lim = tau_max_of(m);
  Vec7 out;
  for (int i = 0; i < kDof; ++i) {
    out(i) = raw(i) > lim(i) ? lim(i) : (raw(i) < -lim(i) ? -lim(i) : raw(i));
  }
  return out;
}

}  // namespace cobot::control::registry_test

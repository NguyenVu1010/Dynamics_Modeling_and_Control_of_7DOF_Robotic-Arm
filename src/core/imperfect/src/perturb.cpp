#include "cobot/imperfect/perturb.hpp"

#include <algorithm>
#include <cstddef>
#include <random>

#include "cobot/common/dof.hpp"

namespace cobot::imperfect {
namespace {

constexpr double kMinMassScale = 0.1;  // lower clamp of the mass scale factor

// Inertia of a point mass m at offset d about the reference point (parallel-axis term).
common::Mat3 point_inertia(double m, const common::Vec3& d) {
  return m * (d.squaredNorm() * common::Mat3::Identity() - d * d.transpose());
}

}  // namespace

template <int N>
model::Model<N> perturb_model(const model::Model<N>& nominal,
                              const ModelMismatchConfig<N>& mismatch,
                              std::uint64_t seed) {
  model::Model<N> out = nominal;
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  // Fixed draw order per link (scale, then com x, y, z) whatever the config, so a link's
  // draws do not depend on which effects are enabled. Effects with zero std are skipped so
  // an all-zero config is bit-identical.
  for (std::size_t k = 0; k < static_cast<std::size_t>(N); ++k) {
    model::LinkInertia& link = out.links[k];
    const double mass_std = mismatch.link_mass_scale_std(static_cast<Eigen::Index>(k));
    const double z_mass = unit(rng);
    if (mass_std > 0.0) {
      const double scale = std::max(kMinMassScale, 1.0 + mass_std * z_mass);
      link.mass_kg *= scale;
      link.I_com_i *= scale;
    }
    common::Vec3 z_com;
    for (int ax = 0; ax < 3; ++ax) {
      z_com(ax) = unit(rng);
    }
    if (mismatch.com_shift_std_m > 0.0) {
      link.com_i += mismatch.com_shift_std_m * z_com;
    }
  }

  if (mismatch.payload_kg > 0.0) {
    model::LinkInertia& last = out.links[static_cast<std::size_t>(N) - 1];
    const common::Vec3& tip = nominal.T_last_tip.p;  // payload at the tip origin, frame N
    const double m_new = last.mass_kg + mismatch.payload_kg;
    const common::Vec3 com_new = (last.mass_kg * last.com_i + mismatch.payload_kg * tip) / m_new;
    last.I_com_i += point_inertia(last.mass_kg, last.com_i - com_new) +
                    point_inertia(mismatch.payload_kg, tip - com_new);
    last.mass_kg = m_new;
    last.com_i = com_new;
  }
  return out;
}

template <int N>
dynamics::PlantEffects<N> plant_effects(const ImperfectionConfig<N>& config) {
  dynamics::PlantEffects<N> fx;
  fx.coulomb_nm = config.friction.coulomb_nm;
  fx.viscous_nm_s_rad = config.friction.viscous_nm_s_rad;
  fx.motor_time_constant_s = config.actuator.motor_time_constant_s;
  return fx;
}

#define COBOT_PERTURB_INSTANTIATE(N)                                             \
  template model::Model<(N)> perturb_model<(N)>(                                 \
      const model::Model<(N)>&, const ModelMismatchConfig<(N)>&, std::uint64_t); \
  template dynamics::PlantEffects<(N)> plant_effects<(N)>(const ImperfectionConfig<(N)>&);
COBOT_INSTANTIATE_DOF(COBOT_PERTURB_INSTANTIATE)
#undef COBOT_PERTURB_INSTANTIATE

}  // namespace cobot::imperfect

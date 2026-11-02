#pragma once

#include <cstdint>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/model/model.hpp"

// Plant-side imperfections (feature `sim-plant`). The controller keeps the nominal model;
// the simulated plant gets the perturbed one, so the controller's model is wrong the way
// a real robot's would be.
namespace cobot::imperfect {

// Masses scaled by (1 + N(0, link_mass_scale_std)) (clamped to >= 0.1), inertia scaled by
// the same factor, centre of mass shifted by N(0, com_shift_std_m) per axis, payload (a solid
// sphere of payload_radius_m centred at payload_offset_m in the tip frame) merged into the last
// link by the parallel-axis theorem. Offset 0 and radius 0 give a point mass at the tip origin;
// payload_kg = 0 ignores offset and radius. Deterministic for a given seed.
template <int N>
model::Model<N> perturb_model(const model::Model<N>& nominal,
                              const ModelMismatchConfig<N>& mismatch,
                              std::uint64_t seed);

// Friction, motor lag and transmission for dynamics::SimPlant from the imperfection config; the
// transmission error phase is drawn uniform in [0, 2 pi) per joint from
// std::mt19937_64(seed ^ 0x7A45'0003).
template <int N>
dynamics::PlantEffects<N> plant_effects(const ImperfectionConfig<N>& config);

}  // namespace cobot::imperfect

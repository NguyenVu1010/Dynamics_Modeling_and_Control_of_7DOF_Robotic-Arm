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
// the same factor, centre of mass shifted by N(0, com_shift_std_m) per axis, payload added
// as a point mass at the tip origin to the last link. Deterministic for a given seed.
template <int N>
model::Model<N> perturb_model(const model::Model<N>& nominal, const ModelMismatchConfig<N>& mismatch,
                              std::uint64_t seed);

// Friction and motor lag for dynamics::SimPlant from the imperfection config.
template <int N>
dynamics::PlantEffects<N> plant_effects(const ImperfectionConfig<N>& config);

}  // namespace cobot::imperfect

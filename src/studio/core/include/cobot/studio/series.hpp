#pragma once

#include <string>
#include <vector>

#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/studio_config.hpp"

#include "run_data.hpp"

// What the studio charts plot (feature studio-core, cobot_studio design 5.1, ADR-021); no Qt.
namespace cobot::studio {

enum class Channel {
  kQRef,
  kQ,
  kQd,
  kQdd,  // qdd_ref
  kQTrue,
  kNoise,
  kError,  // q_ref - q
  kTau,
  kTauMotor,
  kTauFriction,
  kTaskPosErr,  // |p_d - p|
  kTaskRotErr,  // angle of R_d R^T
};

struct Series {
  std::vector<double> t_s, y;
  std::string unavailable;  // reason text when there is no line
};

// One line of `run`: joint 0..N-1, or -1 for the task channels. Noise by source (design 5.1):
// sensor q - q_true; actuator tau_motor[k+1] - tau[k] (taum of row k+1 is the plant's response to
// tau of row k, so rows - 1 points at the times of rows 0..rows-2); friction tau_friction; model,
// timing and comm sources (kNone) have no noise line and give `unavailable`.
Series extract(const bench::RunData& run,
               Channel channel,
               int joint,
               NoiseSource source = NoiseSource::kNone);

// The noise source of the first changed key that has one (catalog), kNone otherwise.
NoiseSource noise_source_of(const StudioConfig& config);

}  // namespace cobot::studio

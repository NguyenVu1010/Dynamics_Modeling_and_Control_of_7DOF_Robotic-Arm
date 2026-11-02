#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <random>

#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"

// Decorator that makes any JointInterface imperfect (feature `imperfect-io`, design 11.1).
// read():  delay -> offset -> noise -> quantisation of q; qd either noisy or from the
//          low-passed difference of the output q.
// write(): scale error -> ripple (phase from the last true q read from the inner interface)
//          -> deadband -> resolution rounding -> command delay -> slew limit -> torque-speed
//          ceiling (peak torque, knee and zero-torque speeds scaled by the bus voltage, which
//          sags with the previous cycle's motoring power; realism-2 design 6.1). Friction and motor
//          lag are plant effects (dynamics::PlantEffects) in bench; on the Gazebo path they are
//          approximated here (friction from the last measured qd, lag as a first-order filter) when
//          `apply_plant_effects` is true.
// Servo bus (comm, realism-2 design 5 and 6.3): a packet on read k (k counted from 0) when
// k % S == 0, S = control rate / servo_rate_hz (1 when 0); one Bernoulli draw per packet loses
// both directions. A received packet processes a fresh sample through the sensor chain and
// forwards the shaped command (after resolution, before the command delay); otherwise read()
// repeats its previous output (same stamp) and the last forwarded command is sent again. A
// latency burst (one draw per read while none is active) makes the next burst_cycles reads
// repeat too. The first read always processes its sample. The inner read() runs every cycle.
// Comm draws come from their own generator (seed ^ kCommSalt), so other streams are unchanged.
// Deterministic for a given seed; no heap allocation after construction.
namespace cobot::imperfect {

using common::Status;
using hal::JointCommand;
using hal::JointInterface;
using hal::JointState;

inline constexpr int kMaxDelayCycles = 32;
inline constexpr std::uint64_t kCommSalt = 0xC0AA'0001;  // seed ^ salt: comm generator

template <int N>
class ImperfectJointInterface final : public JointInterface<N> {
 public:
  // Throws std::invalid_argument when a delay or comm.burst_cycles is outside
  // [0, kMaxDelayCycles], a comm probability outside [0, 1], 1 / (control_period_s *
  // comm.servo_rate_hz) not an integer >= 1,
  // control_period_s <= 0, or a joint has a torque-speed knee with an infinite peak torque
  // or a zero-torque speed not above the knee. `peak_torque_nm` is the P of the curve
  // (hosts pass config.limits.peak_torque_nm); it has no effect without a knee.
  ImperfectJointInterface(JointInterface<N>& inner,
                          const ImperfectionConfig<N>& config,
                          double control_period_s,
                          bool apply_plant_effects = false,
                          const common::VecN<N>& peak_torque_nm =
                              common::VecN<N>::Constant(std::numeric_limits<double>::infinity()));

  Status read(JointState<N>& out) override;
  Status write(const JointCommand<N>& cmd) override;

  // Values drawn once at construction, for reports.
  const common::VecN<N>& q_offset_rad() const { return q_offset_; }
  const common::VecN<N>& torque_scale() const { return torque_scale_; }
  // Bus voltage used by the last write, V; v_nom_v before the first write. With the bus model
  // off (v_nom_v 0) it is the normalised nominal 1.0 (V = V_nom := 1, no scaling), never 0.
  double bus_voltage_v() const { return bus_voltage_v_; }
  // Servo packets lost so far, and the read count of the last lost one (-1 if none).
  std::int64_t packets_lost() const { return packets_lost_; }
  std::int64_t last_lost_cycle() const { return last_lost_cycle_; }

 private:
  // Advances the comm state by one read; true when this read processes a fresh sample.
  bool comm_fresh();

  JointInterface<N>& inner_;
  ImperfectionConfig<N> config_;
  double period_s_;
  bool apply_plant_effects_;
  std::mt19937_64 rng_;
  common::VecN<N> q_offset_;
  common::VecN<N> torque_scale_;
  std::array<JointState<N>, kMaxDelayCycles + 1> state_ring_{};
  std::array<JointCommand<N>, kMaxDelayCycles + 1> cmd_ring_{};
  int state_head_ = 0;
  int cmd_head_ = 0;
  int reads_ = 0;
  int writes_ = 0;
  common::VecN<N> q_prev_out_ = common::VecN<N>::Zero();
  common::VecN<N> qd_filtered_ = common::VecN<N>::Zero();
  common::VecN<N> tau_lagged_ = common::VecN<N>::Zero();
  common::VecN<N> q_last_true_ = common::VecN<N>::Zero();
  common::VecN<N> qd_last_true_ = common::VecN<N>::Zero();  // torque-speed curve and bus
  common::VecN<N> peak_torque_;
  common::VecN<N> tau_prev_out_ = common::VecN<N>::Zero();  // slew and bus, before plant
  double bus_voltage_v_ = 1.0;
  common::VecN<N> qd_last_out_ = common::VecN<N>::Zero();  // friction phase, plant effects
  // Comm state.
  std::mt19937_64 comm_rng_;
  std::int64_t servo_cycles_ = 1;  // S
  std::int64_t cycle_ = 0;         // reads so far
  std::int64_t packets_lost_ = 0;
  std::int64_t last_lost_cycle_ = -1;
  std::int64_t out_cycle_ = 0;  // read count of the last processed sample
  int burst_left_ = 0;
  bool forward_ = true;    // the last read's packet arrived: write() forwards the command
  bool have_out_ = false;  // a sample has been processed
  JointState<N> last_out_{};
  JointCommand<N> held_cmd_{};  // last forwarded command, zeros before the first
};

}  // namespace cobot::imperfect

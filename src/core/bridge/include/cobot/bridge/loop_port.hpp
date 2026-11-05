#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/seqlock.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/manual.hpp"
#include "cobot/pipeline/spsc_ring.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// Control-thread side of the studio bridge in a host (feature controller-bridge, ADR-022): the
// hand-off between the control thread (before_step / after_step around ControlLoop::step) and
// the bridge thread (CommandService), in one process. pipeline::SpscRing instead of
// realtime_tools::RealtimeBuffer, so cobot_controller and cobot_bench serve share it.
namespace cobot::bridge {

template <int N>
struct LoopCommand {  // bridge thread -> control thread; copying never allocates
  enum class Kind : std::uint8_t { kLoad, kMoveJoint, kStop, kEstop, kReset };
  std::uint64_t id = 0;  // the wire Command id (0: posted by the service itself, no response)
  Kind kind = Kind::kStop;
  pipeline::CommandMode cmd = pipeline::CommandMode::kWaypoints;  // kLoad: kWaypoints or kMoveTcp
  std::optional<planning::WaypointTrajectory> trajectory;         // kLoad
  pipeline::JointMove<N> move;                                    // kMoveJoint
};

struct LoopAck {  // control thread -> bridge thread
  std::uint64_t id = 0;
  common::Status status = common::Status::ok();  // a load/reset rejection may carry built text
  std::int64_t t_ns = 0;                         // stamp of the step that applied it
};

struct JogAck {
  std::uint64_t refresh = 0;
  const char* rejected = nullptr;  // literal or null
};

template <int N>
struct LoopStatus {  // trivially copyable snapshot of the last after_step()
  std::int64_t t_ns = 0;
  std::uint8_t mode = 0, cmd = 0, manual_stop = 0, hold_reason = 0;
  std::uint32_t standby_seen = 0;  // standby entries seen (also a new loop in standby)
  double q[N] = {}, joint_reference[N] = {}, standby_q[N] = {};
  WirePose reference_tip{}, standby_reference_tip{};
};

template <int N>
class LoopPort {
 public:
  // endpoint null: no state publishing; commands and jogs need one (the hosts create the port
  // only with an endpoint). Rings: commands 8, acks 16, jog acks 32, allocated here.
  explicit LoopPort(ServerEndpoint<N>* endpoint);

  // Control thread, before ControlLoop::step: applies at most one posted command and pushes its
  // ack; reads the jog slot and, when its refresh changed and its token is the current owner's
  // (!= 0): direction 0 -> request_stop() if a jog runs, else jog_joint / jog_tcp (a rejection ->
  // JogAck with the literal). A torn or stale-token slot is ignored (the deadman then stops a
  // running jog). No allocation when the command is accepted.
  void before_step(pipeline::ControlLoop<N>& loop, const hal::JointState<N>& state) noexcept;
  // Control thread, after step: publishes the state sample and the LoopStatus; captures
  // standby_q (measured q) and standby_reference_tip on entering standby (or on a new loop).
  void after_step(const pipeline::ControlLoop<N>& loop) noexcept;

  // Bridge thread.
  bool post(const LoopCommand<N>& c);  // false when full
  bool take_ack(LoopAck& out);
  bool take_jog_ack(JogAck& out);
  bool status(LoopStatus<N>& out) const;  // false when torn (retry)

 private:
  void apply(pipeline::ControlLoop<N>& loop, const hal::JointState<N>& state) noexcept;
  void jog(pipeline::ControlLoop<N>& loop) noexcept;

  ServerEndpoint<N>* endpoint_;
  std::unique_ptr<pipeline::SpscRing<LoopCommand<N>, 8>> commands_;
  std::unique_ptr<pipeline::SpscRing<LoopAck, 16>> acks_;
  std::unique_ptr<pipeline::SpscRing<JogAck, 32>> jog_acks_;
  std::unique_ptr<SeqSlot<LoopStatus<N>>> status_;
  std::unique_ptr<LoopCommand<N>> command_;  // the popped command (kept off the stack)
  LoopStatus<N> snapshot_{};                 // control thread
  std::uint64_t last_refresh_ = 0;
  std::uint64_t step_ = 0;
  const void* last_loop_ = nullptr;
  bool was_standby_ = false;
};

}  // namespace cobot::bridge

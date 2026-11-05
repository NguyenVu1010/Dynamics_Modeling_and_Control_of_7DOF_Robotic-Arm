#include "cobot/bridge/loop_port.hpp"

#include "cobot/common/dof.hpp"

namespace cobot::bridge {
namespace {

using pipeline::CommandMode;
using pipeline::LoopMode;

// ControlLoop::jog_joint / jog_tcp reject with literals (static storage), so a JogAck carries the
// pointer itself and no text is built on the control thread; built text maps to a fixed literal.
const char* jog_literal(const common::Status& st) noexcept {
  const char* literal = st.literal();
  return literal != nullptr ? literal : "jog: rejected";
}

}  // namespace

template <int N>
LoopPort<N>::LoopPort(ServerEndpoint<N>* endpoint)
    : endpoint_(endpoint),
      commands_(std::make_unique<pipeline::SpscRing<LoopCommand<N>, 8>>()),
      acks_(std::make_unique<pipeline::SpscRing<LoopAck, 16>>()),
      jog_acks_(std::make_unique<pipeline::SpscRing<JogAck, 32>>()),
      status_(std::make_unique<SeqSlot<LoopStatus<N>>>()),
      command_(std::make_unique<LoopCommand<N>>()) {}

template <int N>
void LoopPort<N>::before_step(pipeline::ControlLoop<N>& loop,
                              const hal::JointState<N>& state) noexcept {
  if (commands_->pop(*command_)) {
    apply(loop, state);
  }
  if (endpoint_ != nullptr) {
    jog(loop);
  }
}

template <int N>
void LoopPort<N>::apply(pipeline::ControlLoop<N>& loop, const hal::JointState<N>& state) noexcept {
  using Kind = typename LoopCommand<N>::Kind;
  const LoopCommand<N>& c = *command_;
  LoopAck ack;
  ack.id = c.id;
  ack.t_ns = state.t.ns;
  switch (c.kind) {
    case Kind::kLoad:
      ack.status = c.trajectory ? loop.load(*c.trajectory, c.cmd)
                                : common::Status::error("load: no trajectory");
      break;
    case Kind::kMoveJoint:
      ack.status = loop.move_joint(c.move);
      break;
    case Kind::kStop:
      loop.request_stop();
      break;
    case Kind::kEstop:
      ack.status = loop.estop();
      break;
    case Kind::kReset:
      ack.status = loop.reset(state);
      break;
  }
  static_cast<void>(acks_->push(ack));
}

template <int N>
void LoopPort<N>::jog(pipeline::ControlLoop<N>& loop) noexcept {
  JogRequest j{};
  if (!endpoint_->read_jog(j) || j.refresh == last_refresh_) {
    return;  // torn (retry next step), none, or nothing new
  }
  last_refresh_ = j.refresh;
  const std::uint64_t owner = endpoint_->owner_token();
  if (owner == 0 || j.owner_token != owner) {
    return;  // stale token: ignored
  }
  common::Status st = common::Status::ok();
  if (j.direction == 0) {
    const CommandMode cmd = loop.command_mode();
    if (loop.mode() == LoopMode::kTracking &&
        (cmd == CommandMode::kJogJoint || cmd == CommandMode::kJogTcp)) {
      loop.request_stop();
    }
  } else if (j.kind == 1) {
    st = loop.jog_joint(j.joint, j.direction, j.speed);
  } else if (j.kind == 2) {
    st = loop.jog_tcp(static_cast<pipeline::JogAxis>(j.axis),
                      j.direction,
                      static_cast<pipeline::JogFrame>(j.frame),
                      j.speed);
  }
  if (!st.is_ok()) {
    static_cast<void>(jog_acks_->push(JogAck{j.refresh, jog_literal(st)}));
  }
}

template <int N>
void LoopPort<N>::after_step(const pipeline::ControlLoop<N>& loop) noexcept {
  const pipeline::LogRecord<N>& r = loop.last_record();
  const auto manual_stop = static_cast<std::uint8_t>(loop.manual_stop());
  if (endpoint_ != nullptr) {
    endpoint_->publish(to_sample(r, ++step_, manual_stop));
  }
  LoopStatus<N>& s = snapshot_;
  s.t_ns = r.t.ns;
  s.mode = static_cast<std::uint8_t>(loop.mode());
  s.cmd = static_cast<std::uint8_t>(loop.command_mode());
  s.manual_stop = manual_stop;
  s.hold_reason = static_cast<std::uint8_t>(loop.hold_reason());
  const common::VecN<N>& q_ref = loop.joint_reference();
  for (int i = 0; i < N; ++i) {
    s.q[i] = r.q(i);
    s.joint_reference[i] = q_ref(i);
  }
  s.reference_tip = to_wire(loop.reference_pose());
  const bool standby = loop.mode() == LoopMode::kStandby;
  if (standby && (!was_standby_ || last_loop_ != &loop)) {
    ++s.standby_seen;
    for (int i = 0; i < N; ++i) {
      s.standby_q[i] = r.q(i);
    }
    s.standby_reference_tip = s.reference_tip;
  }
  was_standby_ = standby;
  last_loop_ = &loop;
  status_->write(s);
}

template <int N>
bool LoopPort<N>::post(const LoopCommand<N>& c) {
  return commands_->push(c);
}

template <int N>
bool LoopPort<N>::take_ack(LoopAck& out) {
  return acks_->pop(out);
}

template <int N>
bool LoopPort<N>::take_jog_ack(JogAck& out) {
  return jog_acks_->pop(out);
}

template <int N>
bool LoopPort<N>::status(LoopStatus<N>& out) const {
  return status_->try_read(out);
}

#define COBOT_INSTANTIATE(N) template class LoopPort<N>;
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::bridge

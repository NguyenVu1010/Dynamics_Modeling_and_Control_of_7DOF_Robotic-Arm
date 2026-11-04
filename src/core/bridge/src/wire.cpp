#include "cobot/bridge/wire.hpp"

#include <cstring>

#include "cobot/common/dof.hpp"

namespace cobot::bridge {

WirePose to_wire(const common::Pose& T) {
  WirePose w{};
  for (int r = 0; r < 3; ++r) {
    w.p[r] = T.p(r);
    for (int c = 0; c < 3; ++c) {
      w.R[3 * r + c] = T.R(r, c);
    }
  }
  return w;
}

common::Pose from_wire(const WirePose& w) {
  common::Pose T;
  for (int r = 0; r < 3; ++r) {
    T.p(r) = w.p[r];
    for (int c = 0; c < 3; ++c) {
      T.R(r, c) = w.R[3 * r + c];
    }
  }
  return T;
}

void copy_text(char* dst, std::size_t size, std::string_view text) noexcept {
  static constexpr char kEllipsis[] = "...";
  static constexpr std::size_t kEllipsisChars = sizeof(kEllipsis) - 1;
  if (dst == nullptr || size == 0) {
    return;
  }
  if (text.size() < size) {
    std::memcpy(dst, text.data(), text.size());
    dst[text.size()] = '\0';
    return;
  }
  if (size <= kEllipsisChars) {  // no room for the ellipsis: cut
    std::memcpy(dst, text.data(), size - 1);
    dst[size - 1] = '\0';
    return;
  }
  const std::size_t keep = size - 1 - kEllipsisChars;
  std::memcpy(dst, text.data(), keep);
  std::memcpy(dst + keep, kEllipsis, sizeof(kEllipsis));  // with the terminating 0
}

template <int N>
StateSample<N> to_sample(const pipeline::LogRecord<N>& r,
                         std::uint64_t step,
                         std::uint8_t manual_stop) noexcept {
  StateSample<N> s{};
  s.step = step;
  s.t_ns = r.t.ns;
  s.mode = static_cast<std::uint8_t>(r.mode);
  s.hold_reason = static_cast<std::uint8_t>(r.hold_reason);
  s.cmd_mode = static_cast<std::uint8_t>(r.cmd_mode);
  s.manual_stop = manual_stop;
  for (int i = 0; i < N; ++i) {
    s.q[i] = r.q(i);
    s.qd[i] = r.qd(i);
    s.q_ref[i] = r.q_ref(i);
    s.qd_ref[i] = r.qd_ref(i);
    s.tau[i] = r.tau(i);
    s.q_true[i] = r.q_true(i);
  }
  s.T_b_e = to_wire(r.T_b_e);
  s.T_b_d = to_wire(r.T_b_d);
  for (int k = 0; k < 6; ++k) {
    s.task_error[k] = r.task_error(k);
  }
  s.step_time_us = r.step_time_us;
  return s;
}

#define COBOT_INSTANTIATE(N)            \
  template StateSample<N> to_sample<N>( \
      const pipeline::LogRecord<N>& r, std::uint64_t step, std::uint8_t manual_stop) noexcept;
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::bridge

// The control-thread part of the bridge (LoopPort::before_step + ControlLoop::step +
// LoopPort::after_step, feature controller-bridge, plan "Global constraints") allocates nothing:
// over 1000 steps of a jog driven through the jog slot with state publishing, and on every step of
// an accepted move_joint and of an accepted waypoint load (the steps that apply them included).
// The counters are on only around that part; the bridge thread (poll) and the harness may
// allocate.
#include <cstddef>
#include <cstdlib>
#include <new>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "bridge_service_util.hpp"

// Counts every global operator new in this binary while g_counting is set, so a std::vector,
// std::string or dynamic Eigen matrix created by the bridge or the loop (compiled in their
// libraries, where EIGEN_RUNTIME_NO_MALLOC is not defined) is caught.
namespace {
bool g_counting = false;      // test-only, single thread
std::size_t g_new_calls = 0;  // test-only
}  // namespace

void* operator new(std::size_t size) {
  if (g_counting) {
    ++g_new_calls;
  }
  if (void* p = std::malloc(size == 0 ? 1 : size)) {
    return p;
  }
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
  return ::operator new(size);
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}

// GoogleTest allocates with the nothrow forms; route them through the replacements above so
// every allocation and deallocation pair matches (otherwise ASan reports alloc-dealloc-mismatch).
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return ::operator new(size);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return ::operator new[](size);
  } catch (...) {
    return nullptr;
  }
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept {
  ::operator delete(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept {
  ::operator delete[](p);
}

namespace cobot::bridge::test {
namespace {

// Wraps the control part of one harness step in both allocation guards and adds the operator new
// calls it made to `news`.
struct Guard {
  std::size_t& news;
  template <typename Fn>
  void operator()(const Fn& control) const {
    const std::size_t before = g_new_calls;
    g_counting = true;
    Eigen::internal::set_is_malloc_allowed(false);
    control();
    Eigen::internal::set_is_malloc_allowed(true);
    g_counting = false;
    news += g_new_calls - before;
  }
};

bool in_standby(const Harness& h) {
  return static_cast<LoopMode>(h.status().mode) == LoopMode::kStandby;
}

// Steps with the guard until the response to `id` arrived and the loop is back in standby.
std::optional<Response> run_guarded(
    Harness& h, Client& client, std::uint64_t id, std::size_t& news, int max_steps) {
  std::optional<Response> answer;
  bool left_standby = false;
  for (int k = 0; k < max_steps; ++k) {
    h.step(Guard{news});
    Response r{};
    while (client.poll_response(r)) {
      if (r.command_id == id && r.type != CommandType::kJog) {
        answer = r;
      }
    }
    left_standby = left_standby || !in_standby(h);
    if (answer && left_standby && in_standby(h)) {
      return answer;
    }
  }
  ADD_FAILURE() << "command " << id << " did not run to standby within " << max_steps << " steps";
  return answer;
}

struct Owned {
  Harness h;
  std::optional<Client> client;
  Owned() {
    if (!h.ok()) {
      return;
    }
    h.steps(20);
    client = open_client(h.name());
    if (client) {
      EXPECT_TRUE(client->claim(h.now_ns()).is_ok());
      h.beat_with(&*client);
      h.steps(10);
    }
  }
  bool ok() const { return h.ok() && client.has_value() && client->is_owner(); }
};

TEST(BridgeServiceNoAlloc, JogThroughTheSlotWithPublishingDoesNotAllocate) {
  Owned o;
  ASSERT_TRUE(o.ok());
  std::size_t news = 0;
  for (int k = 0; k < 1000; ++k) {
    if (k % 20 == 0) {
      o.client->write_jog(joint_jog(0, +1, kSpeed));
    }
    o.h.step(Guard{news});
  }
  EXPECT_EQ(static_cast<LoopMode>(o.h.status().mode), LoopMode::kTracking);
  o.client->write_jog(joint_jog(0, 0, kSpeed));
  for (int k = 0; k < 300 && !in_standby(o.h); ++k) {
    o.h.step(Guard{news});
  }
  EXPECT_TRUE(in_standby(o.h));
  EXPECT_EQ(news, 0U);
  // The state ring was written: a client reads samples.
  std::vector<StateSample<kN>> samples;
  samples.reserve(kStateSlots);
  o.client->read_states(samples);
  EXPECT_FALSE(samples.empty());
}

TEST(BridgeServiceNoAlloc, AcceptedMoveJointAndWaypointLoadDoNotAllocateOnTheControlThread) {
  Owned o;
  ASSERT_TRUE(o.ok());
  std::size_t news = 0;

  const std::vector<planning::Waypoint> tcp = e2e_13_waypoints();
  ASSERT_FALSE(tcp.empty());
  const auto run = o.client->send(waypoints_command(CommandType::kRunWaypoints, tcp));
  ASSERT_TRUE(run.is_ok()) << run.reason();
  const auto ran = run_guarded(o.h, *o.client, run.value(), news, 30000);
  ASSERT_TRUE(ran);
  EXPECT_EQ(ran->accepted, 1U) << text_of(*ran);
  EXPECT_EQ(news, 0U) << "waypoint load";

  const Vec goal = move_joint_goal(vec_of(o.h.status().joint_reference));
  const auto move = o.client->send(move_joint_command(goal, kSpeed));
  ASSERT_TRUE(move.is_ok()) << move.reason();
  const auto moved = run_guarded(o.h, *o.client, move.value(), news, 4000);
  ASSERT_TRUE(moved);
  EXPECT_EQ(moved->accepted, 1U) << text_of(*moved);
  EXPECT_EQ(news, 0U) << "move_joint";
}

}  // namespace
}  // namespace cobot::bridge::test

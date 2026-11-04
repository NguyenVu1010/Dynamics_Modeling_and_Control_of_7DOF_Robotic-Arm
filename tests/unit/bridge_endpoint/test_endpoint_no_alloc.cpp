// The control-thread side of the server never allocates (feature shm-bridge, ADR-022, ADR-008):
// publish, read_jog and owner_token run every step. The global operator new is replaced, so this
// is a binary of its own.
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"

#include "bridge_test_util.hpp"
#include "count_new.hpp"

namespace cobot::bridge::test {
namespace {

constexpr std::uint64_t kSteps = 10000;  // > kStateSlots: the ring wraps

TEST(BridgeEndpointNoAlloc, PublishReadJogAndOwnerTokenDoNotAllocate) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto client = Client::open(g.name());
  ASSERT_TRUE(client.is_ok()) << client.reason();
  Server& s = server.value();
  const Sample a = make_sample(1);
  const Sample b = make_sample(2);
  JogRequest out{};
  std::uint64_t owned = 0;
  std::uint64_t jogs = 0;
  {
    // No jog written yet, nobody owns the bridge.
    const count_new::Scope scope;
    for (std::uint64_t k = 0; k < kSteps; ++k) {
      s.publish(k % 2 == 0 ? a : b);
      jogs += s.read_jog(out) ? 1 : 0;
      owned += s.owner_token() != 0 ? 1 : 0;
    }
    EXPECT_EQ(count_new::Scope::count(), 0U);
  }
  EXPECT_EQ(jogs, 0U);
  EXPECT_EQ(owned, 0U);

  ASSERT_TRUE(client.value().claim(monotonic_ns()).is_ok());
  JogRequest j{};
  j.kind = 1;
  j.direction = 1;
  client.value().write_jog(j);
  {
    // An owner and a jog in the slot.
    const count_new::Scope scope;
    for (std::uint64_t k = 0; k < kSteps; ++k) {
      s.publish(k % 2 == 0 ? a : b);
      jogs += s.read_jog(out) ? 1 : 0;
      owned += s.owner_token() != 0 ? 1 : 0;
    }
    EXPECT_EQ(count_new::Scope::count(), 0U);
  }
  EXPECT_EQ(jogs, kSteps);
  EXPECT_EQ(owned, kSteps);
  EXPECT_EQ(out.kind, 1);
}

}  // namespace
}  // namespace cobot::bridge::test

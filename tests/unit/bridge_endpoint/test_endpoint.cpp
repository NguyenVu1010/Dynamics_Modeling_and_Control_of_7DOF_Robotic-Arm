// Server and client endpoints of the studio bridge in one process (feature shm-bridge, cobot_studio
// design 6.5, ADR-022 "Vùng nhớ cầu nối", "Các vòng", "Quyền điều khiển"): region life cycle,
// layout checks, authority, the command, response and state rings, the jog slot, owner state and
// session info. The seqlock load tests are in test_endpoint_load.cpp, the allocation test in
// test_endpoint_no_alloc.cpp, the two-process tests in tests/component/bridge_two_process.
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <type_traits>
#include <unistd.h>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/wire.hpp"

#include "bridge_test_util.hpp"

namespace cobot::bridge::test {
namespace {

static_assert(!std::is_copy_constructible_v<Server>, "ServerEndpoint is move-only");
static_assert(!std::is_copy_assignable_v<Server>, "ServerEndpoint is move-only");
static_assert(std::is_move_constructible_v<Server>, "ServerEndpoint is movable");
static_assert(!std::is_copy_constructible_v<Client>, "ClientEndpoint is move-only");
static_assert(!std::is_copy_assignable_v<Client>, "ClientEndpoint is move-only");
static_assert(std::is_move_constructible_v<Client>, "ClientEndpoint is movable");

constexpr auto kChildTimeout = std::chrono::milliseconds(30000);

std::string our_pid() {
  return std::to_string(static_cast<long>(::getpid()));
}

// A command whose payload is tagged by `k`: q_goal[i] = k * 10 + i, path_a "cfg_<k>.yaml".
Cmd tagged_command(std::uint64_t k) {
  Cmd c{};
  c.id = 999;           // overwritten by send
  c.owner_token = 999;  // overwritten by send
  c.type = CommandType::kMoveJoint;
  c.speed = 0.5;
  for (int i = 0; i < kDof; ++i) {
    c.q_goal[i] = static_cast<double>(k) * 10.0 + i;
  }
  copy_text(c.path_a, kPathChars, "cfg_" + std::to_string(k) + ".yaml");
  return c;
}

// ---------------------------------------------------------------- clock

TEST(BridgeClock, MonotonicNsIsClockMonotonic) {
  timespec a{};
  timespec c{};
  ::clock_gettime(CLOCK_MONOTONIC, &a);
  const std::int64_t b = monotonic_ns();
  ::clock_gettime(CLOCK_MONOTONIC, &c);
  const std::int64_t a_ns = static_cast<std::int64_t>(a.tv_sec) * 1000000000 + a.tv_nsec;
  const std::int64_t c_ns = static_cast<std::int64_t>(c.tv_sec) * 1000000000 + c.tv_nsec;
  EXPECT_LE(a_ns, b);
  EXPECT_LE(b, c_ns);
}

// ---------------------------------------------------------------- region life cycle

TEST(BridgeRegion, CreateThenOpenRoundTrip) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  EXPECT_TRUE(g.present()) << g.region();
  auto client = Client::open(g.name());
  ASSERT_TRUE(client.is_ok()) << client.reason();

  Session s{};
  ASSERT_TRUE(client.value().session(s));
  EXPECT_EQ(std::string(s.config_path), "/tmp/config_1.yaml");
  EXPECT_EQ(std::string(s.imperfections_path), "/tmp/imperfections_1.yaml");
  EXPECT_EQ(s.seed, 1001U);
  EXPECT_EQ(s.control_rate_hz, 1000.0);
  EXPECT_EQ(s.q_max[6], 8.0);  // 1 + 6 + 1

  server.value().publish(make_sample(1));
  std::vector<Sample> got;
  EXPECT_EQ(client.value().read_states(got), 1U);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].step, 1U);
  EXPECT_TRUE(sample_intact(got[0]));
  EXPECT_EQ(client.value().lost(), 0U);
}

TEST(BridgeRegion, SecondCreateFromTheSameProcessNamesOurPid) {
  const RegionGuard g;
  auto first = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(first.is_ok()) << first.reason();
  const auto second = Server::create(g.name(), make_session(2));
  ASSERT_FALSE(second.is_ok());
  // "bridge /cobot_<name> is served by pid <p>"
  EXPECT_TRUE(contains(second.reason(), g.region())) << second.reason();
  EXPECT_TRUE(contains(second.reason(), "served by pid " + our_pid())) << second.reason();

  // The first server and its session are untouched.
  auto client = Client::open(g.name());
  ASSERT_TRUE(client.is_ok()) << client.reason();
  Session s{};
  ASSERT_TRUE(client.value().session(s));
  EXPECT_EQ(std::string(s.config_path), "/tmp/config_1.yaml");
  first.value().publish(make_sample(1));
  std::vector<Sample> got;
  EXPECT_EQ(client.value().read_states(got), 1U);
}

// A region left by a server that is gone (its pid reaped) is unlinked and created afresh.
TEST(BridgeRegion, StaleRegionOfAReapedServerIsReplaced) {
  const RegionGuard g;
  const std::string& name = g.name();
  Child child(fork_child([&name] {
    auto server = Server::create(name, make_session(2));
    if (!server.is_ok()) {
      return 3;
    }
    for (std::uint64_t k = 1; k <= 5; ++k) {
      server.value().publish(make_sample(k));
    }
    return 0;
  }));
  ASSERT_GT(child.pid(), 0);
  ASSERT_EQ(child.wait_exit(kChildTimeout), 0);
  ASSERT_TRUE(g.present()) << "the child's region outlives it";

  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto client = Client::open(g.name());
  ASSERT_TRUE(client.is_ok()) << client.reason();
  Session s{};
  ASSERT_TRUE(client.value().session(s));
  EXPECT_EQ(std::string(s.config_path), "/tmp/config_1.yaml");
  std::vector<Sample> got;
  EXPECT_EQ(client.value().read_states(got), 0U) << "fresh state ring";
  EXPECT_EQ(client.value().lost(), 0U);
}

TEST(BridgeRegion, DestructorUnmapsButNeverUnlinks) {
  const RegionGuard g;
  {
    auto created = Server::create(g.name(), make_session(1));
    ASSERT_TRUE(created.is_ok()) << created.reason();
    const Server moved = std::move(created.value());  // move-only: the moved-to object owns it
    static_cast<void>(moved.owner_token());
  }
  EXPECT_TRUE(g.present());
  const auto client = Client::open(g.name());
  EXPECT_TRUE(client.is_ok()) << client.reason();
}

TEST(BridgeRegion, RemoveTwiceIsOkAndOpenThenSaysNoBridge) {
  const RegionGuard g;
  ASSERT_TRUE(Server::create(g.name(), make_session(1)).is_ok());
  const common::Status first = Server::remove(g.name());
  EXPECT_TRUE(first.is_ok()) << first.reason();
  EXPECT_FALSE(g.present());
  const common::Status second = Server::remove(g.name());
  EXPECT_TRUE(second.is_ok()) << "missing is ok: " << second.reason();
  const auto client = Client::open(g.name());
  ASSERT_FALSE(client.is_ok());
  EXPECT_TRUE(contains(client.reason(), "no bridge " + g.region())) << client.reason();
}

TEST(BridgeRegion, OpenOfAMissingNameSaysNoBridge) {
  const RegionGuard g;
  const auto client = Client::open(g.name());
  ASSERT_FALSE(client.is_ok());
  // "no bridge /cobot_<name>"
  EXPECT_TRUE(contains(client.reason(), "no bridge " + g.region())) << client.reason();
  EXPECT_FALSE(g.present()) << "open never creates";
}

TEST(BridgeRegion, BadNamesAreErrors) {
  const std::vector<std::string> bad = {
      "", "a-b", "a/b", "a b", "../x", "a.b", std::string(65, 'a')};
  for (const std::string& name : bad) {
    const auto r = Server::create(name, make_session(1));
    EXPECT_FALSE(r.is_ok()) << "name '" << name << "'";
    if (!name.empty() && name.find('/') == std::string::npos) {
      static_cast<void>(::shm_unlink(region_of(name).c_str()));
    }
    EXPECT_FALSE(Client::open(name).is_ok()) << "name '" << name << "'";
  }
}

// The region holds the three rings (and is reserved up front with posix_fallocate); its size is
// reported for the PR body (expected ~4 MB for N = 7) and fits the 64 MB /dev/shm of the arm64
// build stage.
TEST(BridgeRegion, RegionHoldsTheRingsAndFitsTheBuildStage) {
  const RegionGuard g;
  ASSERT_TRUE(Server::create(g.name(), make_session(1)).is_ok());
  const std::size_t need = kStateSlots * sizeof(Sample) + kCommandSlots * sizeof(Cmd) +
                           kResponseSlots * sizeof(Response);
  struct stat st {};
  ASSERT_EQ(::stat(("/dev/shm/cobot_" + g.name()).c_str(), &st), 0);
  const auto bytes = static_cast<std::size_t>(st.st_size);
  ::testing::Test::RecordProperty("region_bytes", std::to_string(bytes));
  std::printf("region /cobot_<name> for N = 7: %zu bytes (%.2f MB)\n",
              bytes,
              static_cast<double>(bytes) / (1024.0 * 1024.0));
  EXPECT_GE(bytes, need);
  EXPECT_LT(bytes, std::size_t{64} * 1024 * 1024);
  // tmpfs counts reserved pages in st_blocks (512-byte units).
  EXPECT_GE(static_cast<std::size_t>(st.st_blocks) * 512, need) << "pages not reserved";
}

// ---------------------------------------------------------------- layout checks

// A fresh region whose server object is already gone (the region stays, its pid is ours).
void make_region(const RegionGuard& g) {
  ASSERT_TRUE(Server::create(g.name(), make_session(1)).is_ok());
}

// Opens the region of `g` after writing `value` (4 bytes) at the identity field `field` picks.
template <typename Pick>
std::string open_after_patch(const RegionGuard& g, Pick field, std::uint32_t value) {
  {
    RawRegion raw(g.name());
    if (!raw.ok()) {
      ADD_FAILURE() << "raw mapping of " << g.region();
      return {};
    }
    const std::optional<std::size_t> off = field(find_identity(raw));
    if (!off) {
      ADD_FAILURE() << "field not found in the header (see find_identity)";
      return {};
    }
    raw.write(*off, value);
  }
  const auto r = Client::open(g.name());
  EXPECT_FALSE(r.is_ok()) << "open accepted a patched header";
  return r.is_ok() ? std::string() : r.reason();
}

TEST(BridgeLayout, FreshRegionOpensAndItsHeaderIsFound) {
  const RegionGuard g;
  make_region(g);
  {
    const RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    EXPECT_EQ(raw.read<std::uint64_t>(0), kBridgeMagic) << "magic first";
    const Identity id = find_identity(raw);
    EXPECT_TRUE(id.version && id.dof && id.state_size && id.command_size && id.response_size &&
                id.state_slots && id.command_slots && id.response_slots)
        << "identity fields in the documented order within " << kIdentityBytes << " bytes";
  }
  const auto r = Client::open(g.name());
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(BridgeLayout, CorruptMagicIsRejected) {
  const RegionGuard g;
  make_region(g);
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    raw.bytes()[0] ^= 0xFFU;
  }
  const auto r = Client::open(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "magic")) << r.reason();
}

TEST(BridgeLayout, OtherLayoutVersionIsRejectedNamingBothValues) {
  const RegionGuard g;
  make_region(g);
  const std::uint32_t v = kBridgeLayoutVersion;
  const std::string why = open_after_patch(g, [](const Identity& id) { return id.version; }, v + 1);
  EXPECT_TRUE(contains(lower(why), "layout version")) << why;
  EXPECT_TRUE(contains(why, std::to_string(v + 1))) << why;
  EXPECT_TRUE(contains(why, std::to_string(v))) << why;
}

TEST(BridgeLayout, OtherDofIsRejectedNamingBothValues) {
  const RegionGuard g;
  ASSERT_TRUE(ServerEndpoint<2>::create(g.name(), SessionInfo<2>{}).is_ok());
  const auto r = Client::open(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "dof: region 2, client 7")) << r.reason();

  // And the other way round.
  const RegionGuard h;
  make_region(h);
  const auto back = ClientEndpoint<2>::open(h.name());
  ASSERT_FALSE(back.is_ok());
  EXPECT_TRUE(contains(back.reason(), "dof: region 7, client 2")) << back.reason();
}

// The error names the record ("state", "command", "response"), says "size" and gives both values.
TEST(BridgeLayout, OtherRecordSizesAreRejectedNamingBothValues) {
  struct Case {
    const char* word;
    std::size_t size;
    std::optional<std::size_t> (*pick)(const Identity&);
  };
  const Case cases[] = {
      {"state", sizeof(Sample), [](const Identity& id) { return id.state_size; }},
      {"command", sizeof(Cmd), [](const Identity& id) { return id.command_size; }},
      {"response", sizeof(Response), [](const Identity& id) { return id.response_size; }},
  };
  for (const Case& c : cases) {
    const RegionGuard g;
    make_region(g);
    const auto size = static_cast<std::uint32_t>(c.size);
    const std::string why = open_after_patch(g, c.pick, size + 8);
    const std::string low = lower(why);
    EXPECT_TRUE(contains(low, c.word) && contains(low, "size")) << c.word << ": " << why;
    EXPECT_TRUE(contains(why, std::to_string(size + 8))) << c.word << ": " << why;
    EXPECT_TRUE(contains(why, std::to_string(size))) << c.word << ": " << why;
  }
}

// The error names the ring ("state", "command", "response"), says "slot" or "capacity" and gives
// both values.
TEST(BridgeLayout, OtherRingCapacitiesAreRejectedNamingBothValues) {
  struct Case {
    const char* word;
    std::size_t slots;
    std::optional<std::size_t> (*pick)(const Identity&);
  };
  const Case cases[] = {
      {"state", kStateSlots, [](const Identity& id) { return id.state_slots; }},
      {"command", kCommandSlots, [](const Identity& id) { return id.command_slots; }},
      {"response", kResponseSlots, [](const Identity& id) { return id.response_slots; }},
  };
  for (const Case& c : cases) {
    const RegionGuard g;
    make_region(g);
    const auto slots = static_cast<std::uint32_t>(c.slots);
    const std::string why = open_after_patch(g, c.pick, slots + 1);
    const std::string low = lower(why);
    EXPECT_TRUE(contains(low, c.word)) << c.word << ": " << why;
    EXPECT_TRUE(contains(low, "slot") || contains(low, "capacit")) << c.word << ": " << why;
    EXPECT_TRUE(contains(why, std::to_string(slots + 1))) << c.word << ": " << why;
    EXPECT_TRUE(contains(why, std::to_string(slots))) << c.word << ": " << why;
  }
}

// Checks run in the order magic, version, N, sizes, capacities: with both the version and the
// response size wrong, the version is named.
TEST(BridgeLayout, VersionIsCheckedBeforeTheSizes) {
  const RegionGuard g;
  make_region(g);
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const Identity id = find_identity(raw);
    ASSERT_TRUE(id.version && id.response_size);
    raw.write(*id.response_size, static_cast<std::uint32_t>(sizeof(Response) + 8));
    raw.write(*id.version, static_cast<std::uint32_t>(kBridgeLayoutVersion + 1));
  }
  const auto r = Client::open(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "layout version")) << r.reason();
  EXPECT_FALSE(contains(lower(r.reason()), "response")) << r.reason();
}

// ---------------------------------------------------------------- authority

TEST(BridgeAuthority, ClaimMakesTheClientTheOwner) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto opened = Client::open(g.name());
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  Client& c = opened.value();
  const std::int64_t now = monotonic_ns();
  EXPECT_FALSE(c.is_owner());
  EXPECT_EQ(server.value().owner_token(), 0U);
  EXPECT_EQ(server.value().owner_state(now), OwnerState::kNone);

  const common::Status claimed = c.claim(now);
  ASSERT_TRUE(claimed.is_ok()) << claimed.reason();
  EXPECT_TRUE(c.is_owner());
  EXPECT_EQ(c.owner_pid(), static_cast<std::int64_t>(::getpid()));
  const std::uint64_t token = server.value().owner_token();
  EXPECT_NE(token, 0U);
  EXPECT_EQ(server.value().owner_state(now), OwnerState::kAlive);

  // Claiming twice from the same client is ok and keeps the authority.
  const common::Status again = c.claim(now);
  EXPECT_TRUE(again.is_ok()) << again.reason();
  EXPECT_TRUE(c.is_owner());
  EXPECT_EQ(server.value().owner_token(), token);

  c.release();
  EXPECT_FALSE(c.is_owner());
  EXPECT_EQ(server.value().owner_token(), 0U);
  EXPECT_EQ(server.value().owner_state(now), OwnerState::kNone);
}

TEST(BridgeAuthority, SecondClientIsRefusedUntilReleaseAndTheOldOwnerIsLockedOut) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto a = Client::open(g.name());
  auto b = Client::open(g.name());
  ASSERT_TRUE(a.is_ok() && b.is_ok());
  const std::int64_t now = monotonic_ns();
  ASSERT_TRUE(a.value().claim(now).is_ok());
  const std::uint64_t token_a = server.value().owner_token();

  const common::Status refused = b.value().claim(now);
  ASSERT_FALSE(refused.is_ok());
  EXPECT_TRUE(contains(refused.reason(), "held by pid " + our_pid())) << refused.reason();
  EXPECT_FALSE(b.value().is_owner());
  EXPECT_EQ(b.value().owner_pid(), static_cast<std::int64_t>(::getpid()));

  // release() of a non-owner is a no-op.
  b.value().release();
  EXPECT_TRUE(a.value().is_owner());
  EXPECT_EQ(server.value().owner_token(), token_a);

  a.value().release();
  const common::Status taken = b.value().claim(now);
  ASSERT_TRUE(taken.is_ok()) << taken.reason();
  EXPECT_TRUE(b.value().is_owner());
  EXPECT_FALSE(a.value().is_owner());
  const std::uint64_t token_b = server.value().owner_token();
  EXPECT_NE(token_b, 0U);
  EXPECT_NE(token_b, token_a) << "tokens are random per client";

  // A command of the old owner after the takeover is refused before it reaches the ring.
  const common::Result<std::uint64_t> sent = a.value().send(tagged_command(1));
  ASSERT_FALSE(sent.is_ok());
  EXPECT_TRUE(contains(sent.reason(), "not the owner")) << sent.reason();
  Cmd popped{};
  EXPECT_FALSE(server.value().pop_command(popped));
}

TEST(BridgeAuthority, SendWithoutClaimIsNotTheOwner) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  const common::Result<std::uint64_t> sent = c.value().send(tagged_command(1));
  ASSERT_FALSE(sent.is_ok());
  EXPECT_TRUE(contains(sent.reason(), "not the owner")) << sent.reason();
}

// ---------------------------------------------------------------- command and response rings

TEST(BridgeCommandRing, SixtyFourFitThe65thIsFullAndPopsComeInOrder) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  ASSERT_TRUE(c.value().claim(monotonic_ns()).is_ok());
  const std::uint64_t token = server.value().owner_token();

  for (std::uint64_t k = 1; k <= kCommandSlots; ++k) {
    const common::Result<std::uint64_t> sent = c.value().send(tagged_command(k));
    ASSERT_TRUE(sent.is_ok()) << "send " << k << ": " << sent.reason();
    EXPECT_EQ(sent.value(), k) << "ids 1, 2, ...";
  }
  const common::Result<std::uint64_t> full = c.value().send(tagged_command(65));
  ASSERT_FALSE(full.is_ok());
  EXPECT_TRUE(contains(full.reason(), "command ring full")) << full.reason();

  for (std::uint64_t k = 1; k <= kCommandSlots; ++k) {
    Cmd got{};
    ASSERT_TRUE(server.value().pop_command(got)) << "pop " << k;
    EXPECT_EQ(got.id, k);
    EXPECT_EQ(got.owner_token, token);
    EXPECT_EQ(got.type, CommandType::kMoveJoint);
    EXPECT_EQ(got.speed, 0.5);
    const Cmd want = tagged_command(k);
    for (int i = 0; i < kDof; ++i) {
      EXPECT_EQ(got.q_goal[i], want.q_goal[i]) << "command " << k << " joint " << i;
    }
    EXPECT_EQ(std::string(got.path_a), std::string(want.path_a));
  }
  Cmd none{};
  EXPECT_FALSE(server.value().pop_command(none)) << "empty after 64 pops";

  // Room again once popped.
  EXPECT_TRUE(c.value().send(tagged_command(66)).is_ok());
  EXPECT_TRUE(server.value().pop_command(none));
  EXPECT_EQ(none.q_goal[0], 660.0);
}

TEST(BridgeResponseRing, SixteenFitThe17thIsDroppedAndCounted) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  Server& s = server.value();
  EXPECT_EQ(s.dropped_responses(), 0U);

  const auto response = [](std::uint64_t k) {
    Response r{};
    r.command_id = k;
    r.type = CommandType::kMoveTcp;
    r.accepted = static_cast<std::uint32_t>(k % 2);
    copy_text(r.text, kTextChars, "response " + std::to_string(k));
    r.duration_s = 0.25 * static_cast<double>(k);
    r.path_count = 1;
    r.path[kPreviewPoints - 1][2] = static_cast<double>(k);
    return r;
  };
  for (std::uint64_t k = 1; k <= kResponseSlots; ++k) {
    EXPECT_TRUE(s.push_response(response(k))) << "push " << k;
  }
  EXPECT_FALSE(s.push_response(response(17)));
  EXPECT_EQ(s.dropped_responses(), 1U);

  for (std::uint64_t k = 1; k <= kResponseSlots; ++k) {
    Response got{};
    ASSERT_TRUE(c.value().poll_response(got)) << "poll " << k;
    EXPECT_EQ(got.command_id, k);
    EXPECT_EQ(got.type, CommandType::kMoveTcp);
    EXPECT_EQ(got.accepted, static_cast<std::uint32_t>(k % 2));
    EXPECT_EQ(std::string(got.text), "response " + std::to_string(k));
    EXPECT_EQ(got.duration_s, 0.25 * static_cast<double>(k));
    EXPECT_EQ(got.path[kPreviewPoints - 1][2], static_cast<double>(k));
  }
  Response none{};
  EXPECT_FALSE(c.value().poll_response(none));
  EXPECT_TRUE(s.push_response(response(18)));
  EXPECT_EQ(s.dropped_responses(), 1U);
}

// ---------------------------------------------------------------- jog slot

TEST(BridgeJog, ServerReadsTheLatestJogWithTheOwnersToken) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  JogRequest out{};
  EXPECT_FALSE(server.value().read_jog(out)) << "no jog written yet";

  ASSERT_TRUE(c.value().claim(monotonic_ns()).is_ok());
  JogRequest j{};
  j.kind = 1;
  j.joint = 2;
  j.direction = 1;
  j.speed = 0.1;
  c.value().write_jog(j);
  ASSERT_TRUE(server.value().read_jog(out));
  EXPECT_EQ(out.owner_token, server.value().owner_token());
  EXPECT_EQ(out.refresh, 1U);
  EXPECT_EQ(out.kind, 1);
  EXPECT_EQ(out.joint, 2);
  EXPECT_EQ(out.direction, 1);
  EXPECT_EQ(out.speed, 0.1);

  j.direction = 0;  // release
  c.value().write_jog(j);
  ASSERT_TRUE(server.value().read_jog(out));
  EXPECT_EQ(out.refresh, 2U);
  EXPECT_EQ(out.direction, 0);
}

// ---------------------------------------------------------------- state ring

TEST(BridgeStateRing, ReaderSeesTheNewest4096AndCountsTheRestLost) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  constexpr std::uint64_t kFirst = 10000;
  for (std::uint64_t k = 1; k <= kFirst; ++k) {
    server.value().publish(make_sample(k));
  }
  std::vector<Sample> got;
  // 10000 published, 4096 slots: steps 10000 - 4096 + 1 = 5905 .. 10000 remain, 5904 are lost.
  ASSERT_EQ(c.value().read_states(got), kStateSlots);
  ASSERT_EQ(got.size(), kStateSlots);
  for (std::size_t i = 0; i < got.size(); ++i) {
    ASSERT_EQ(got[i].step, 5905U + i) << "position " << i;
    ASSERT_TRUE(sample_intact(got[i])) << "step " << got[i].step;
  }
  EXPECT_EQ(c.value().lost(), 5904U);

  // 10 more: exactly those, appended after the first 4096; lost unchanged.
  for (std::uint64_t k = kFirst + 1; k <= kFirst + 10; ++k) {
    server.value().publish(make_sample(k));
  }
  ASSERT_EQ(c.value().read_states(got), 10U);
  ASSERT_EQ(got.size(), kStateSlots + 10);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(got[kStateSlots + i].step, kFirst + 1 + i);
    EXPECT_TRUE(sample_intact(got[kStateSlots + i]));
  }
  EXPECT_EQ(c.value().lost(), 5904U);
  EXPECT_EQ(c.value().read_states(got), 0U) << "nothing new";
}

TEST(BridgeStateRing, ExactlyAFullRingLosesNothing) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  for (std::uint64_t k = 1; k <= kStateSlots; ++k) {
    server.value().publish(make_sample(k));
  }
  std::vector<Sample> got;
  ASSERT_EQ(c.value().read_states(got), kStateSlots);
  EXPECT_EQ(got.front().step, 1U);
  EXPECT_EQ(got.back().step, kStateSlots);
  EXPECT_EQ(c.value().lost(), 0U);
}

// ---------------------------------------------------------------- owner state and heartbeats

TEST(BridgeOwnerState, HeartbeatAgeDecidesAliveOrLost) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto a = Client::open(g.name());
  auto b = Client::open(g.name());
  ASSERT_TRUE(a.is_ok() && b.is_ok());
  const std::int64_t t0 = monotonic_ns();
  ASSERT_TRUE(a.value().claim(t0).is_ok());
  a.value().beat(t0);
  const Server& s = server.value();
  EXPECT_EQ(s.owner_state(t0 + 199 * kMs), OwnerState::kAlive);
  EXPECT_EQ(s.owner_state(t0 + 201 * kMs), OwnerState::kHeartbeatLost);

  // A beat of a client that is not the owner changes nothing.
  b.value().beat(t0 + 150 * kMs);
  EXPECT_EQ(s.owner_state(t0 + 201 * kMs), OwnerState::kHeartbeatLost);

  // The owner's next beat: age 201 - 150 = 51 ms.
  a.value().beat(t0 + 150 * kMs);
  EXPECT_EQ(s.owner_state(t0 + 201 * kMs), OwnerState::kAlive);
  EXPECT_EQ(s.owner_state(t0 + 351 * kMs), OwnerState::kHeartbeatLost);
}

TEST(BridgeOwnerState, ReapedOwnerProcessIsGone) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  const Pipe pipe;
  ASSERT_TRUE(pipe.ok());
  const std::string& name = g.name();
  Child child(fork_child([&name, &pipe] {
    auto c = Client::open(name);
    if (!c.is_ok()) {
      return 4;
    }
    const std::int64_t t = monotonic_ns();
    if (!c.value().claim(t).is_ok()) {
      return 3;
    }
    if (!pipe.put(t)) {
      return 5;
    }
    ::_exit(0);  // no destructor runs: the authority is never released
  }));
  ASSERT_GT(child.pid(), 0);
  std::int64_t claimed_at = 0;
  ASSERT_TRUE(pipe.get_within(claimed_at, 30000))
      << "child exit " << child.wait_exit(kChildTimeout);
  ASSERT_EQ(child.wait_exit(kChildTimeout), 0);

  EXPECT_NE(server.value().owner_token(), 0U);
  // Heartbeat 1 ms old, but the pid is gone (kill(pid, 0) == ESRCH).
  EXPECT_EQ(server.value().owner_state(claimed_at + kMs), OwnerState::kProcessGone);
}

TEST(BridgeOwnerState, ClearOwnerDropsTheOwner) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto a = Client::open(g.name());
  auto b = Client::open(g.name());
  ASSERT_TRUE(a.is_ok() && b.is_ok());
  const std::int64_t now = monotonic_ns();
  ASSERT_TRUE(a.value().claim(now).is_ok());
  ASSERT_TRUE(a.value().is_owner());

  server.value().clear_owner();
  EXPECT_EQ(server.value().owner_token(), 0U);
  EXPECT_EQ(server.value().owner_state(now), OwnerState::kNone);
  EXPECT_FALSE(a.value().is_owner());
  const common::Result<std::uint64_t> sent = a.value().send(tagged_command(1));
  ASSERT_FALSE(sent.is_ok());
  EXPECT_TRUE(contains(sent.reason(), "not the owner")) << sent.reason();
  EXPECT_TRUE(b.value().claim(now).is_ok()) << "free again";
}

TEST(BridgeServerBeat, ServerAliveFollowsTheServerHeartbeat) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  const std::int64_t t0 = monotonic_ns();
  server.value().beat(t0);
  EXPECT_TRUE(c.value().server_alive(t0 + 199 * kMs));
  EXPECT_FALSE(c.value().server_alive(t0 + 201 * kMs));
  server.value().beat(t0 + 200 * kMs);
  EXPECT_TRUE(c.value().server_alive(t0 + 201 * kMs));
}

// ---------------------------------------------------------------- session

TEST(BridgeSession, SetSessionTwiceGivesGenerationTwoAndTheSecondContent) {
  const RegionGuard g;
  auto server = Server::create(g.name(), make_session(1));
  ASSERT_TRUE(server.is_ok()) << server.reason();
  auto c = Client::open(g.name());
  ASSERT_TRUE(c.is_ok()) << c.reason();
  server.value().set_session(make_session(2));
  server.value().set_session(make_session(3));
  Session s{};
  ASSERT_TRUE(c.value().session(s));
  EXPECT_EQ(s.generation, 2U);
  const Session want = make_session(3);
  EXPECT_EQ(std::string(s.config_path), "/tmp/config_3.yaml");
  EXPECT_EQ(std::string(s.imperfections_path), "/tmp/imperfections_3.yaml");
  EXPECT_EQ(s.seed, 1003U);
  EXPECT_EQ(s.command_source_external, 1U);
  EXPECT_EQ(s.soft_margin_rad, want.soft_margin_rad);
  EXPECT_EQ(s.jog_margin_rad, want.jog_margin_rad);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_EQ(s.q_min[i], want.q_min[i]) << i;
    EXPECT_EQ(s.q_max[i], want.q_max[i]) << i;
    EXPECT_EQ(s.qd_max[i], want.qd_max[i]) << i;
    EXPECT_EQ(s.tau_max[i], want.tau_max[i]) << i;
    EXPECT_EQ(s.rated[i], want.rated[i]) << i;
  }
  EXPECT_EQ(s.T_tip_tcp.p[2], want.T_tip_tcp.p[2]);
}

}  // namespace
}  // namespace cobot::bridge::test

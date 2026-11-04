#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

// Wire types of the studio bridge (feature shm-bridge, cobot_studio design 6.5, ADR-022 "Bản ghi
// trên dây là POD"). Every type placed in the region is trivially copyable and standard layout;
// fields, units and sizes are documented in docs/conventions.md "Studio bridge".
namespace cobot::bridge {

inline constexpr std::uint64_t kBridgeMagic = 0x3147445242544F43ULL;  // "COTBRDG1"
inline constexpr std::uint32_t kBridgeLayoutVersion = 1;  // bump on any wire/header change
inline constexpr std::size_t kStateSlots = 4096;          // ~4 s at 1 kHz
inline constexpr std::size_t kCommandSlots = 64;  // usable entries (counters, not "one slot empty")
inline constexpr std::size_t kResponseSlots = 16;
inline constexpr std::size_t kWireWaypoints = 32;
inline constexpr std::size_t kWireIssues = 4 * kWireWaypoints;
inline constexpr std::size_t kPreviewPoints = 512;
inline constexpr std::size_t kTextChars = 256, kPathChars = 512;
inline constexpr std::int64_t kHeartbeatPeriodNs = 50'000'000;    // client beats every 50 ms
inline constexpr std::int64_t kHeartbeatTimeoutNs = 200'000'000;  // owner lost after 200 ms

static_assert(kWireWaypoints == planning::kMaxWaypoints, "one wire waypoint per planner waypoint");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "atomics in shared memory");

struct WirePose {  // R row-major; tip or TCP as each field says
  double p[3];
  double R[9];
};
WirePose to_wire(const common::Pose& T);
common::Pose from_wire(const WirePose& w);

// Flat copy of the LogRecord fields the live view needs (ADR-022: not LogRecord itself, which is
// not trivially copyable). Enum fields carry the pipeline enum's integer value.
template <int N>
struct StateSample {
  std::uint64_t step = 0;  // 1, 2, ... per publish (gaps = lost on the reader side)
  std::int64_t t_ns = 0;
  // LoopMode, HoldReason, CommandMode, ManualStop
  std::uint8_t mode = 0, hold_reason = 0, cmd_mode = 0, manual_stop = 0;
  std::uint8_t pad[4] = {};
  double q[N] = {}, qd[N] = {}, q_ref[N] = {}, qd_ref[N] = {}, tau[N] = {}, q_true[N] = {};
  WirePose T_b_e{}, T_b_d{};  // tip poses
  double task_error[6] = {};
  double step_time_us = 0.0;
};

// No allocation; `manual_stop` is passed by the host (the record has no such field).
template <int N>
StateSample<N> to_sample(const pipeline::LogRecord<N>& r,
                         std::uint64_t step,
                         std::uint8_t manual_stop) noexcept;

enum class CommandType : std::uint32_t {
  kMoveJoint = 1,
  kMoveTcp,
  kRunWaypoints,
  kPreviewWaypoints,
  kStop,
  kEstop,
  kReset,
  kApplyConfig,
  kExportSegment,
  kJog = 100  // responses only: a jog refresh the control thread rejected
};

struct WireWaypoint {
  WirePose T_b_tcp{};
  double duration_s = 0.0;
  std::uint32_t stop = 0, pad = 0;
};

template <int N>
struct Command {
  std::uint64_t id = 0;           // set by ClientEndpoint::send (1, 2, ...)
  std::uint64_t owner_token = 0;  // set by send
  CommandType type{};
  std::uint32_t waypoint_count = 0;
  double speed = 0.0;                           // (0, 1]
  double q_goal[N] = {};                        // kMoveJoint, rad
  WirePose T_b_tcp{};                           // kMoveTcp
  WireWaypoint waypoints[kWireWaypoints] = {};  // kRunWaypoints, kPreviewWaypoints (TCP poses)
  std::int64_t t0_ns = 0, t1_ns = 0;            // kExportSegment
  char path_a[kPathChars] = {}, path_b[kPathChars] = {};  // kApplyConfig: config, imperfections
};

struct WireIssue {
  std::uint32_t waypoint = 0, issue = 0;  // issue = pipeline::PathIssue value
  std::int32_t joint = -1;
  std::uint32_t pad = 0;
  double value = 0.0, limit = 0.0;
};

struct Response {
  std::uint64_t command_id = 0;  // kJog: the jog refresh number
  CommandType type{};
  std::uint32_t accepted = 0;
  char text[kTextChars] = {};   // reason, or a short success detail
  std::int64_t t_start_ns = 0;  // stamp of the step that applied the command (motion commands)
  double duration_s = 0.0;      // planned duration (moves, waypoints)
  std::uint32_t issue_count = 0, segment_count = 0, path_count = 0, truncated = 0;
  WireIssue issues[kWireIssues] = {};
  double segment_durations_s[kWireWaypoints] = {};
  double path[kPreviewPoints][3] = {};  // desired TCP positions, evenly decimated, last kept
  char out_path[kPathChars] = {};       // kExportSegment
  std::uint64_t rows = 0;
};

struct JogRequest {  // the jog seqlock slot (client writes, control thread reads)
  std::uint64_t owner_token = 0;
  std::uint64_t refresh = 0;         // incremented by every write; the server acts on a change
  std::int32_t kind = 0;             // 0 none, 1 joint, 2 tcp
  std::int32_t joint = -1;           // 0-based
  std::int32_t axis = 0, frame = 0;  // pipeline::JogAxis / JogFrame values
  std::int32_t direction = 0;        // -1, +1; 0 = release
  std::int32_t pad = 0;
  double speed = 0.0;
};

template <int N>
struct SessionInfo {  // written by the server at start and after "apply config"
  std::uint64_t generation = 0;
  char config_path[kPathChars] = {}, imperfections_path[kPathChars] = {};
  std::uint64_t seed = 0;
  std::uint32_t command_source_external = 0, pad = 0;
  double control_rate_hz = 0.0, soft_margin_rad = 0.0, jog_margin_rad = 0.0;
  // Controller model / cobot.limits.
  double q_min[N] = {}, q_max[N] = {}, qd_max[N] = {}, tau_max[N] = {}, rated[N] = {};
  WirePose T_tip_tcp{};
};

// Copies into a char field with a terminating 0; longer text ends with "..." (never
// unterminated).
void copy_text(char* dst, std::size_t size, std::string_view text) noexcept;

template <typename T>
inline constexpr bool kIsWireType = std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T>;

static_assert(kIsWireType<WirePose> && kIsWireType<WireWaypoint> && kIsWireType<WireIssue> &&
                  kIsWireType<Response> && kIsWireType<JogRequest>,
              "ADR-022: wire types are POD");
static_assert(kIsWireType<StateSample<1>> && kIsWireType<StateSample<2>> &&
                  kIsWireType<StateSample<7>>,
              "ADR-022: wire types are POD");
static_assert(kIsWireType<Command<1>> && kIsWireType<Command<2>> && kIsWireType<Command<7>>,
              "ADR-022: wire types are POD");
static_assert(kIsWireType<SessionInfo<1>> && kIsWireType<SessionInfo<2>> &&
                  kIsWireType<SessionInfo<7>>,
              "ADR-022: wire types are POD");

}  // namespace cobot::bridge

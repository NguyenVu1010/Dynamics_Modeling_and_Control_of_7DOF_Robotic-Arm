#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cobot/bridge/endpoint.hpp"
#include "cobot/bridge/loop_port.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/result.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/path_validation.hpp"

// Bridge-thread side of the studio bridge in a host (feature controller-bridge, cobot_studio
// design 5.4 and 6.5, ADR-022): authority and heartbeat watchdog, validation of the studio's
// commands off the control thread, responses, segment export and apply config. Hosts call poll()
// every <= 5 ms on a normal (non real-time) thread.
namespace cobot::bridge {

template <int N>
class CommandService {
 public:
  struct ExportInfo {
    std::filesystem::path csv;
    std::uint64_t size_after = 0;
  };
  struct Hooks {
    // Appends the Recorder's unexported rows to the host's Run CSV (exactly once) and returns
    // the file and its size after the append.
    std::function<common::Result<ExportInfo>()> export_log;
    // Builds a new runtime from the two files off the control thread, hands it to the control
    // thread and returns the new controller model and config. Empty -> "apply config: not
    // supported by this host".
    std::function<common::Result<std::pair<model::Model<N>, pipeline::Config<N>>>(
        const std::filesystem::path& config, const std::filesystem::path& imperfections)>
        apply_config;
    // Optional: the imperfection seed the host uses after a successful apply_config (its
    // override, else the new file's seed), written to the session. Empty: the seed is kept.
    std::function<std::uint64_t()> seed;
  };

  CommandService(ServerEndpoint<N>& endpoint,
                 LoopPort<N>& port,
                 model::Model<N> model,
                 pipeline::Config<N> config,
                 SessionInfo<N> session,
                 std::filesystem::path segment_dir,
                 Hooks hooks);

  // One pass, in order: server heartbeat, owner watchdog, the popped commands, the loop's acks
  // and jog acks. Non-blocking except the export / apply hooks.
  void poll(std::int64_t now_ns);
  bool studio_in_control() const;  // an owner token is registered

 private:
  struct Pending {                 // a command posted to the loop, answered on its ack
    std::uint64_t sequence = 0;    // LoopCommand::id of the post, echoed by the LoopAck
    std::uint64_t command_id = 0;  // the wire Command id
    CommandType type{};
    double duration_s = 0.0;
  };
  struct Mark {  // Run CSV size after an export, and the loop's stamp read after it
    std::int64_t t_ns = 0;
    std::uint64_t size = 0;
  };

  void watchdog(std::int64_t now_ns);
  void handle(const Command<N>& c);
  void answer_acks();
  // Response skeleton for `c`, rejected with `text` (accepted 0).
  Response& reply(const Command<N>& c, const std::string& text);
  void push();
  bool motion_allowed(const Command<N>& c, const LoopStatus<N>& st);
  void post_motion(const Command<N>& c, LoopCommand<N>& lc, double duration_s);
  void post_pending(const Command<N>& c, LoopCommand<N>& lc, double duration_s);
  void plan_waypoints(const Command<N>& c, const LoopStatus<N>& st);
  void issues_into(const std::vector<pipeline::WaypointIssue>& issues);
  void export_segment(const Command<N>& c);
  void apply_config(const Command<N>& c, const LoopStatus<N>& st);
  void export_mark();
  void add_mark(std::uint64_t size_after);
  bool read_status(LoopStatus<N>& out) const;

  ServerEndpoint<N>& endpoint_;
  LoopPort<N>& port_;
  model::Model<N> model_;
  pipeline::Config<N> config_;
  SessionInfo<N> session_;
  std::filesystem::path segment_dir_;
  Hooks hooks_;
  std::int64_t period_ns_ = 0;
  std::unique_ptr<Command<N>> command_;   // the popped command
  std::unique_ptr<Response> response_;    // the response being built
  std::unique_ptr<LoopCommand<N>> post_;  // the command being posted
  std::vector<Pending> pending_;
  std::vector<Mark> marks_;  // the last kMarks export marks, oldest first
  std::uint64_t last_owner_ = 0;
  std::uint64_t sequence_ = 0;  // the last post's sequence number
  bool stop_wanted_ = false;    // the owner was lost / changed during a studio motion
  bool stop_posted_ = false;    // ... and the watchdog's kStop is in the command ring
  bool studio_motion_ = false;  // the loop tracks a motion the studio started
};

}  // namespace cobot::bridge

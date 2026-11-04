#include "cobot/studio/run_store.hpp"

#include <algorithm>
#include <utility>

namespace cobot::studio {

std::uint64_t RunStore::add(RunEntry entry) {
  entry.id = next_id_++;
  runs_.push_back(std::move(entry));
  return runs_.back().id;
}

bool RunStore::remove(std::uint64_t id) {
  const auto it =
      std::find_if(runs_.begin(), runs_.end(), [id](const RunEntry& e) { return e.id == id; });
  if (it == runs_.end()) {
    return false;
  }
  runs_.erase(it);
  return true;
}

void RunStore::clear() {
  runs_.clear();
}

const RunEntry* RunStore::find(std::uint64_t id) const {
  const auto it =
      std::find_if(runs_.begin(), runs_.end(), [id](const RunEntry& e) { return e.id == id; });
  return it == runs_.end() ? nullptr : &*it;
}

const std::vector<RunEntry>& RunStore::runs() const {
  return runs_;
}

std::size_t RunStore::memory_bytes() const {
  std::size_t bytes = 0;
  for (const RunEntry& e : runs_) {
    bytes += e.data ? e.data->records.capacity() * sizeof(e.data->records.front()) : 0;
  }
  return bytes;
}

}  // namespace cobot::studio

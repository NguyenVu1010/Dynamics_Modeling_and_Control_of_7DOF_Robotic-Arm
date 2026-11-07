#include "cobot/control/algorithm.hpp"

#include <cstddef>

namespace cobot::control {

const char* to_string(Algorithm a) {
  const auto i = static_cast<std::size_t>(a);
  return i < kAlgorithms.size() ? kAlgorithms[i].name : "unknown";
}

std::optional<Algorithm> algorithm_from_string(std::string_view s) {
  for (const AlgorithmInfo& a : kAlgorithms) {
    if (s == a.name) {
      return a.id;
    }
  }
  return std::nullopt;
}

std::string registered_algorithm_names() {
  std::string out;
  for (const AlgorithmInfo& a : kAlgorithms) {
    out += out.empty() ? "" : ", ";
    out += a.name;
  }
  return out;
}

}  // namespace cobot::control

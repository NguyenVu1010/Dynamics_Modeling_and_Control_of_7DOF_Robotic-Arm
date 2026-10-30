#pragma once

#include <cstdint>

namespace cobot::common {

// Time in integer nanoseconds. Compare and sort with the integer
// operators below; convert to double only through seconds_between() to compute dt.
struct Timestamp {
  std::int64_t ns{0};

  constexpr bool operator==(const Timestamp& o) const { return ns == o.ns; }
  constexpr bool operator!=(const Timestamp& o) const { return ns != o.ns; }
  constexpr bool operator<(const Timestamp& o) const { return ns < o.ns; }
  constexpr bool operator<=(const Timestamp& o) const { return ns <= o.ns; }
  constexpr bool operator>(const Timestamp& o) const { return ns > o.ns; }
  constexpr bool operator>=(const Timestamp& o) const { return ns >= o.ns; }
};

constexpr std::int64_t kNanosecondsPerSecond = 1000000000;

// dt in seconds from `from` to `to`. Negative if `to` precedes `from`.
constexpr double seconds_between(Timestamp from, Timestamp to) {
  return static_cast<double>(to.ns - from.ns) / static_cast<double>(kNanosecondsPerSecond);
}

constexpr Timestamp timestamp_from_sec_nsec(std::int64_t sec, std::int64_t nsec) {
  return Timestamp{sec * kNanosecondsPerSecond + nsec};
}

}  // namespace cobot::common

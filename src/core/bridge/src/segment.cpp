#include "cobot/bridge/segment.hpp"

#include <charconv>
#include <fstream>
#include <ios>
#include <string>
#include <system_error>

namespace cobot::bridge {

common::Result<SegmentCopy> copy_csv_range(const std::filesystem::path& run_csv,
                                           std::uint64_t offset,
                                           std::int64_t t0_ns,
                                           std::int64_t t1_ns,
                                           std::int64_t period_ns,
                                           const std::filesystem::path& out_csv) {
  using Out = common::Result<SegmentCopy>;
  std::ifstream in(run_csv, std::ios::binary);
  std::string line;
  if (!in || !std::getline(in, line) || in.eof()) {
    return Out::error("segment: cannot read the header of " + run_csv.string());
  }
  std::ofstream out(out_csv, std::ios::binary | std::ios::trunc);
  if (!out) {
    return Out::error("segment: cannot write " + out_csv.string());
  }
  out << line << '\n';
  if (offset > 0) {
    in.seekg(static_cast<std::streamoff>(offset));
  }
  SegmentCopy copy;
  // A last line without '\n' is a row still being appended: not copied.
  while (std::getline(in, line) && !in.eof()) {
    std::int64_t t = 0;
    const char* end = line.data() + line.size();
    if (std::from_chars(line.data(), end, t).ec != std::errc() || t < t0_ns || t > t1_ns) {
      continue;
    }
    if (copy.rows == 0) {
      copy.first_t_ns = t;
    }
    copy.last_t_ns = t;
    ++copy.rows;
    out << line << '\n';
  }
  out.close();
  if (copy.rows == 0) {
    std::error_code ec;
    std::filesystem::remove(out_csv, ec);
    return Out::error("segment: no row in [" + std::to_string(t0_ns) + ", " +
                      std::to_string(t1_ns) + "] of " + run_csv.string());
  }
  if (!out) {
    return Out::error("segment: cannot write " + out_csv.string());
  }
  copy.truncated = copy.first_t_ns > t0_ns + period_ns;
  return Out::ok(copy);
}

}  // namespace cobot::bridge

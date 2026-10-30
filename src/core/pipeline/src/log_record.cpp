#include "cobot/pipeline/log_record.hpp"

#include <array>
#include <cstddef>
#include <cstdio>
#include <ostream>

#include <Eigen/Geometry>

#include "cobot/common/dof.hpp"

namespace cobot::pipeline {
namespace {

using common::Mat3;
using common::Vec3;

constexpr std::size_t kCellChars = 40;  // "%.17g" of a double needs at most 24 characters

void put(std::ostream& os, double v) {
  std::array<char, kCellChars> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "%.17g", v);
  os.write(buf.data(), n);
}

template <int N>
void put_all(std::ostream& os, const VecN<N>& v) {
  for (int i = 0; i < N; ++i) {
    os.put(',');
    put(os, v[i]);
  }
}

void put_names(std::ostream& os, const char* prefix, int count) {
  for (int i = 1; i <= count; ++i) {
    os << ',' << prefix << i;
  }
}

// Rotation vector of R via Eigen::AngleAxisd; equal to common::so3_log for angles < pi
// (kept local so this writer does not depend on the Lie-group helpers).
Vec3 rotation_vector(const Mat3& R) {
  const Eigen::AngleAxisd aa(R);
  const Vec3 r = aa.angle() * aa.axis();
  return r;
}

}  // namespace

const char* to_string(LoopMode mode) {
  constexpr std::array<const char*, 6> kNames = {
      "idle", "tracking", "holding", "finished", "fault", "standby"};
  return kNames[static_cast<std::size_t>(mode)];
}

template <int N>
void write_csv_header(std::ostream& os) {
  os << "t_ns,mode";
  put_names(os, "q", N);
  put_names(os, "qd", N);
  put_names(os, "qref", N);
  put_names(os, "qdref", N);
  put_names(os, "tau", N);
  os << ",px,py,pz,rx,ry,rz,pdx,pdy,pdz,rdx,rdy,rdz";
  put_names(os, "e", 6);
  os << ",step_us\n";
}

template <int N>
void write_csv_row(std::ostream& os, const LogRecord<N>& record) {
  os << record.t.ns << ',' << to_string(record.mode);
  put_all<N>(os, record.q);
  put_all<N>(os, record.qd);
  put_all<N>(os, record.q_ref);
  put_all<N>(os, record.qd_ref);
  put_all<N>(os, record.tau);
  put_all<3>(os, record.T_b_e.p);
  put_all<3>(os, rotation_vector(record.T_b_e.R));
  put_all<3>(os, record.T_b_d.p);
  put_all<3>(os, rotation_vector(record.T_b_d.R));
  put_all<6>(os, record.task_error);
  os.put(',');
  put(os, record.step_time_us);
  os.put('\n');
}

#define COBOT_INSTANTIATE(N)                            \
  template void write_csv_header<N>(std::ostream & os); \
  template void write_csv_row<N>(std::ostream & os, const LogRecord<N>& record);
COBOT_INSTANTIATE_DOF(COBOT_INSTANTIATE)
#undef COBOT_INSTANTIATE

}  // namespace cobot::pipeline

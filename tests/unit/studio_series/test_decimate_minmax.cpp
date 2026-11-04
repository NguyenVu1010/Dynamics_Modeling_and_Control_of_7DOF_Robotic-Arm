#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#if __has_include("cobot/studio/series.hpp")
#include "cobot/studio/series.hpp"
#elif __has_include("series.hpp")
#include "series.hpp"
#endif

// Min/max decimation for plotting (feature studio-app, cobot_studio design 5.3, ADR-021), declared
// in the studio-core header series.hpp. Redeclared here so this test pins the signature:
//
// Keeps the first and last sample of [t0, t1] and, per pixel column of [t0, t1], the samples with
// the min and max y in time order. At most 2 * pixels + 2 points; every point is an input sample
// (t[i], y[i]). pixels <= 0 gives an empty result.
namespace cobot::studio {
// NOLINTNEXTLINE(readability-redundant-declaration): pins the signature declared in series.hpp
std::vector<std::pair<double, double>> decimate_minmax(
    const std::vector<double>& t, const std::vector<double>& y, double t0, double t1, int pixels);
}  // namespace cobot::studio

namespace cobot::studio {
namespace {

using Points = std::vector<std::pair<double, double>>;

constexpr std::size_t kSamples = 1000000;
constexpr std::size_t kSpikeIndex = 500000;  // t = 500000 / 1e6 = 0.5 exactly
constexpr double kSpikeHeight = 5.0;         // sin is within [-1, 1], so the spike is the max
constexpr double kPi = 3.14159265358979323846;

// t_i = i / 1e6 on [0, 0.999999]; y = sin(2 pi 3 t) plus a one-sample spike at t = 0.5.
struct Signal {
  std::vector<double> t, y;
};

const Signal& signal() {
  static const Signal s = [] {
    Signal out;
    out.t.resize(kSamples);
    out.y.resize(kSamples);
    for (std::size_t i = 0; i < kSamples; ++i) {
      out.t[i] = static_cast<double>(i) / 1e6;
      out.y[i] = std::sin(2.0 * kPi * 3.0 * out.t[i]);
    }
    out.y[kSpikeIndex] += kSpikeHeight;
    return out;
  }();
  return s;
}

bool contains(const Points& points, double t, double y) {
  return std::find(points.begin(), points.end(), std::make_pair(t, y)) != points.end();
}

// Every point must be an input sample: t is uniform with step 1e-6, so its index is t * 1e6.
void expect_input_samples(const Points& points) {
  const Signal& s = signal();
  for (const auto& [t, y] : points) {
    const auto i = static_cast<std::size_t>(std::llround(t * 1e6));
    ASSERT_LT(i, kSamples) << "t " << t;
    EXPECT_EQ(s.t[i], t);
    EXPECT_EQ(s.y[i], y) << "t " << t;
  }
}

void expect_time_order(const Points& points) {
  for (std::size_t k = 1; k < points.size(); ++k) {
    EXPECT_LE(points[k - 1].first, points[k].first) << "point " << k;
  }
}

TEST(DecimateMinmax, FullRangeKeepsExtremesSpikeAndEnds) {
  const Signal& s = signal();
  constexpr int kPixels = 800;
  const Points points = decimate_minmax(s.t, s.y, s.t.front(), s.t.back(), kPixels);

  // At most 2 * 800 + 2 = 1602 points.
  ASSERT_FALSE(points.empty());
  EXPECT_LE(points.size(), 1602U);
  expect_time_order(points);
  expect_input_samples(points);

  EXPECT_EQ(points.front(), std::make_pair(s.t.front(), s.y.front()));
  EXPECT_EQ(points.back(), std::make_pair(s.t.back(), s.y.back()));

  const auto min_it = std::min_element(s.y.begin(), s.y.end());
  const auto max_it = std::max_element(s.y.begin(), s.y.end());
  const auto i_min = static_cast<std::size_t>(std::distance(s.y.begin(), min_it));
  const auto i_max = static_cast<std::size_t>(std::distance(s.y.begin(), max_it));
  // The spike (sin(3 pi) + 5 ~= 5) is the global max.
  EXPECT_EQ(i_max, kSpikeIndex);
  EXPECT_TRUE(contains(points, s.t[i_min], s.y[i_min])) << "global min at t " << s.t[i_min];
  EXPECT_TRUE(contains(points, s.t[i_max], s.y[i_max])) << "global max at t " << s.t[i_max];
  EXPECT_TRUE(contains(points, 0.5, s.y[kSpikeIndex])) << "spike at t 0.5";
}

TEST(DecimateMinmax, SubRangeKeepsOnlyThatRange) {
  const Signal& s = signal();
  constexpr double kT0 = 0.2;
  constexpr double kT1 = 0.4;
  constexpr int kPixels = 100;
  const Points points = decimate_minmax(s.t, s.y, kT0, kT1, kPixels);

  // At most 2 * 100 + 2 = 202 points, all inside [0.2, 0.4].
  ASSERT_FALSE(points.empty());
  EXPECT_LE(points.size(), 202U);
  expect_time_order(points);
  expect_input_samples(points);
  for (const auto& [t, y] : points) {
    EXPECT_GE(t, kT0);
    EXPECT_LE(t, kT1);
    // The spike at t = 0.5 is outside the range: every kept y is a plain sine value.
    EXPECT_LE(std::abs(y), 1.0) << "t " << t;
  }

  // First and last samples of the range (the step is 1e-6 s, so they lie within one step of the
  // range ends).
  std::size_t first = 0;
  while (s.t[first] < kT0) {
    ++first;
  }
  std::size_t last = first;
  while (last + 1 < kSamples && s.t[last + 1] <= kT1) {
    ++last;
  }
  EXPECT_LT(s.t[first] - kT0, 1.5e-6);
  EXPECT_LT(kT1 - s.t[last], 1.5e-6);
  EXPECT_EQ(points.front(), std::make_pair(s.t[first], s.y[first]));
  EXPECT_EQ(points.back(), std::make_pair(s.t[last], s.y[last]));

  // Min and max of the range, found from the samples: sin(6 pi t) on [0.2, 0.4] runs from
  // sin(1.2 pi) = -0.588 down to -1 at t = 0.25 (6 pi t = 1.5 pi), then up to sin(2.4 pi) = 0.951
  // at the right end (the next peak, t = 2.5 / 6 = 0.4167, is outside).
  const auto begin = s.y.begin() + static_cast<std::ptrdiff_t>(first);
  const auto end = s.y.begin() + static_cast<std::ptrdiff_t>(last) + 1;
  const auto i_min =
      static_cast<std::size_t>(std::distance(s.y.begin(), std::min_element(begin, end)));
  const auto i_max =
      static_cast<std::size_t>(std::distance(s.y.begin(), std::max_element(begin, end)));
  EXPECT_TRUE(contains(points, s.t[i_min], s.y[i_min])) << "range min at t " << s.t[i_min];
  EXPECT_TRUE(contains(points, s.t[i_max], s.y[i_max])) << "range max at t " << s.t[i_max];
}

TEST(DecimateMinmax, NonPositivePixelsGiveEmpty) {
  const Signal& s = signal();
  EXPECT_TRUE(decimate_minmax(s.t, s.y, s.t.front(), s.t.back(), 0).empty());
  EXPECT_TRUE(decimate_minmax(s.t, s.y, s.t.front(), s.t.back(), -5).empty());
}

TEST(DecimateMinmax, EmptyInputGivesEmpty) {
  const std::vector<double> none;
  EXPECT_TRUE(decimate_minmax(none, none, 0.0, 1.0, 800).empty());
}

// Fewer samples than pixel columns: nothing to drop, every sample is kept in order.
TEST(DecimateMinmax, ShortSeriesKeptWhole) {
  const std::vector<double> t = {0.0, 0.1, 0.2, 0.3, 0.4};
  const std::vector<double> y = {1.0, -2.0, 3.0, -4.0, 5.0};
  Points points = decimate_minmax(t, y, 0.0, 0.4, 800);
  // A column holding one sample may report it as both its min and its max: drop repeats.
  points.erase(std::unique(points.begin(), points.end()), points.end());
  ASSERT_EQ(points.size(), t.size());
  for (std::size_t i = 0; i < t.size(); ++i) {
    EXPECT_EQ(points[i], std::make_pair(t[i], y[i])) << "sample " << i;
  }
}

}  // namespace
}  // namespace cobot::studio

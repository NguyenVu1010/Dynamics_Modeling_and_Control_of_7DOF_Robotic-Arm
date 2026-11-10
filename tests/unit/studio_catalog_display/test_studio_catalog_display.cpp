// Presentation fields of the parameter catalog (feature studio-theme-catalog, studio config UI
// design 3.2): every entry has a short English label and description, a display unit and scale
// chosen so that the values of the six profiles under config/ read between 0.1 and 1000, the xyz
// flag of model.payload_offset_m, and format_display (three significant digits, no exponent).
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/overrides.hpp"
#include "cobot/studio/param_catalog.hpp"
#include "cobot/studio/profile_catalog.hpp"

namespace cobot::studio {
namespace {

constexpr std::size_t kMaxLabel = 28;
constexpr std::size_t kMaxDescription = 90;
constexpr double kRoundTripRel = 1e-12;
constexpr double kDisplayMin = 0.1;
constexpr double kDisplayMax = 1000.0;

// Characters (UTF-8 code points), not bytes, so a label with "µ" is not counted twice.
std::size_t utf8_length(const std::string& s) {
  std::size_t n = 0;
  for (const char c : s) {
    if ((static_cast<unsigned char>(c) & 0xC0U) != 0x80U) {
      ++n;
    }
  }
  return n;
}

bool is_numeric(const ParamInfo& p) {
  return p.kind == ParamKind::kNumber || p.kind == ParamKind::kPerJoint;
}

// The entries of a kNumber (one) or kPerJoint (a list) base value.
std::vector<double> numbers(const pipeline::KeyOverride& o) {
  if (const auto* d = std::get_if<double>(&o.value)) {
    return {*d};
  }
  if (const auto* v = std::get_if<std::vector<double>>(&o.value)) {
    return *v;
  }
  return {};
}

std::vector<BasePair> pairs() {
  return discover_base_pairs(COBOT_REPO_DIR "/config");
}

TEST(StudioCatalogDisplay, EveryEntryHasAShortLabelAndDescription) {
  ASSERT_FALSE(param_catalog().empty());
  for (const ParamInfo& p : param_catalog()) {
    SCOPED_TRACE(p.key);
    EXPECT_FALSE(p.label.empty());
    EXPECT_FALSE(p.description.empty());
    EXPECT_LE(utf8_length(p.label), kMaxLabel) << p.label;
    EXPECT_LE(utf8_length(p.description), kMaxDescription) << p.description;
  }
}

TEST(StudioCatalogDisplay, LabelsAreUnique) {
  std::set<std::string> seen;
  for (const ParamInfo& p : param_catalog()) {
    EXPECT_TRUE(seen.insert(p.label).second)
        << "label '" << p.label << "' of " << p.key << " is used twice";
  }
}

TEST(StudioCatalogDisplay, ScaleIsPositiveAndDisplayUnitEmptyExactlyWhenUnitIs) {
  for (const ParamInfo& p : param_catalog()) {
    SCOPED_TRACE(p.key);
    EXPECT_GT(p.display_scale, 0.0);
    EXPECT_TRUE(std::isfinite(p.display_scale));
    EXPECT_EQ(p.display_unit.empty(), p.unit.empty())
        << "unit '" << p.unit << "', display_unit '" << p.display_unit << "'";
  }
}

TEST(StudioCatalogDisplay, PinnedDisplayUnits) {
  struct Row {
    const char* key;
    const char* unit;
    double scale;
  };
  // "µrad": micro sign U+00B5, UTF-8 C2 B5 (brief: "µrad").
  const std::vector<Row> rows = {
      {"sensor.q_noise_std_rad", "µrad", 1e6},
      {"sensor.q_offset_std_rad", "mrad", 1e3},
      {"sensor.qd_noise_std_rad_s", "mrad/s", 1e3},
      {"config.computed_torque.omega", "rad/s", 1.0},
      {"config.safety.hal_timeout_ms", "ms", 1.0},
      {"model.payload_kg", "kg", 1.0},
  };
  for (const Row& r : rows) {
    SCOPED_TRACE(r.key);
    const ParamInfo* p = find_param(r.key);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->display_unit, r.unit);
    EXPECT_DOUBLE_EQ(p->display_scale, r.scale);
  }
}

TEST(StudioCatalogDisplay, XyzOnlyForThePayloadOffset) {
  for (const ParamInfo& p : param_catalog()) {
    EXPECT_EQ(p.xyz, p.key == "model.payload_offset_m") << p.key;
  }
  const ParamInfo* offset = find_param("model.payload_offset_m");
  ASSERT_NE(offset, nullptr);
  EXPECT_TRUE(offset->xyz);
}

TEST(StudioCatalogDisplay, FormatDisplayExamples) {
  // Three significant digits, trailing zeros dropped. 1.745 is stored as 1.74500000000000010658,
  // so it rounds up to "1.75".
  EXPECT_EQ(format_display(1.745), "1.75");
  EXPECT_EQ(format_display(5.0), "5");
  EXPECT_EQ(format_display(0.0005), "0.0005");
  EXPECT_EQ(format_display(0.0), "0");
  EXPECT_EQ(format_display(-2.5), "-2.5");
  // |value| >= 1000: rounded to an integer, never an exponent (50000 is not "5e+04").
  EXPECT_EQ(format_display(50000.0), "50000");
  EXPECT_EQ(format_display(1000.0), "1000");
  EXPECT_EQ(format_display(1234.6), "1235");
}

TEST(StudioCatalogDisplay, FormatDisplayMoreValues) {
  // 0.1234 -> 3 significant digits 0.123; 123.456 -> 123; 12.0 -> "12" (zeros dropped);
  // 0.5 -> "0.5"; 7200 >= 1000 -> integer "7200"; -1234.6 -> magnitude rule, "-1235".
  EXPECT_EQ(format_display(0.1234), "0.123");
  EXPECT_EQ(format_display(123.456), "123");
  EXPECT_EQ(format_display(12.0), "12");
  EXPECT_EQ(format_display(0.5), "0.5");
  EXPECT_EQ(format_display(7200.0), "7200");
  EXPECT_EQ(format_display(-1234.6), "-1235");
}

TEST(StudioCatalogDisplay, DisplayScaleRoundTripsEveryBaseValue) {
  const std::vector<BasePair> base = pairs();
  ASSERT_EQ(base.size(), 6U) << "six profiles under config/imperfections";
  std::size_t checked = 0;
  for (const BasePair& pair : base) {
    const std::map<std::string, pipeline::KeyOverride> values = base_values(pair);
    for (const ParamInfo& p : param_catalog()) {
      if (!is_numeric(p)) {
        continue;
      }
      const auto it = values.find(p.key);
      ASSERT_NE(it, values.end()) << pair.name << " has no base value of " << p.key;
      for (const double x : numbers(it->second)) {
        const double back = (x * p.display_scale) / p.display_scale;
        EXPECT_NEAR(back, x, kRoundTripRel * std::abs(x)) << pair.name << " " << p.key;
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 0U);
}

// Keys whose profile values may lie outside [0.1, 1000] once displayed, each with its reason.
enum class Exception {
  kIntegerCount,  // a count; the value must be a non-negative integer
  kFraction,      // a probability or another unitless fraction; the YAML value must be in [0, 1]
  kStiffness,     // a very large stiffness; any finite value
  kWideRange,     // spans more than any decimal-prefixed unit holds in [0.1, 1000]
};

const std::map<std::string, Exception>& exceptions() {
  static const std::map<std::string, Exception> table = {
      // Integer counts (bits, cycles): 0 means off, and a count has no smaller unit.
      {"sensor.encoder_bits", Exception::kIntegerCount},
      {"sensor.sensor_delay_cycles", Exception::kIntegerCount},
      {"actuator.command_delay_cycles", Exception::kIntegerCount},
      {"timing.overrun_cycles", Exception::kIntegerCount},
      {"comm.burst_cycles", Exception::kIntegerCount},
      // Probabilities (1e-5 .. 1e-3 in the profiles): unitless, so display_unit stays "".
      {"timing.overrun_prob", Exception::kFraction},
      {"comm.packet_loss_prob", Exception::kFraction},
      {"comm.latency_burst_prob", Exception::kFraction},
      // Unitless fractions in [0, 1] (0.005 .. 0.15 in the profiles): display_unit stays "".
      {"actuator.torque_scale_std", Exception::kFraction},
      {"actuator.torque_ripple_frac", Exception::kFraction},
      {"actuator.torque_scale_asym_frac", Exception::kFraction},
      {"model.link_mass_scale_std", Exception::kFraction},
      {"transmission.damping_ratio", Exception::kFraction},
      // Very large stiffnesses: transmission 4700 .. 57000 N m/rad; the position gain
      // kp = omega^2 is 400 (budget) .. 6400 (default.yaml) 1/s^2.
      {"transmission.stiffness_k1_nm_rad", Exception::kStiffness},
      {"transmission.stiffness_k2_nm_rad", Exception::kStiffness},
      {"transmission.stiffness_k3_nm_rad", Exception::kStiffness},
      {"config.computed_torque.kp", Exception::kStiffness},
      // 0.05 (premium_fr3) .. 1.5 (pessimistic) N m s/rad: scale 1 shows 0.05, scale 1e3 shows
      // 1500, so no decimal-prefixed unit holds both ends.
      {"friction.viscous_nm_s_rad", Exception::kWideRange},
  };
  return table;
}

// Empty when `x` (YAML units) is acceptable for `p`, else the reason.
std::string violation(const ParamInfo& p, double x) {
  std::ostringstream out;
  const auto it = exceptions().find(p.key);
  if (it == exceptions().end()) {
    const double shown = std::abs(x * p.display_scale);
    if (shown != 0.0 && (shown < kDisplayMin || shown > kDisplayMax)) {
      out << "shown as " << x * p.display_scale << " " << p.display_unit;
    }
    return out.str();
  }
  switch (it->second) {
    case Exception::kIntegerCount:
      if (x < 0.0 || std::floor(x) != x) {
        out << "count " << x << " is not a non-negative integer";
      }
      break;
    case Exception::kFraction:
      if (x < 0.0 || x > 1.0) {
        out << "fraction " << x << " is outside [0, 1]";
      }
      break;
    case Exception::kStiffness:
    case Exception::kWideRange:
      if (!std::isfinite(x * p.display_scale)) {
        out << "shown as " << x * p.display_scale;
      }
      break;
  }
  return out.str();
}

TEST(StudioCatalogDisplay, ProfileValuesReadBetweenATenthAndAThousand) {
  std::ostringstream failures;
  std::size_t count = 0;
  for (const BasePair& pair : pairs()) {
    const std::map<std::string, pipeline::KeyOverride> values = base_values(pair);
    for (const ParamInfo& p : param_catalog()) {
      if (!is_numeric(p)) {
        continue;
      }
      const auto it = values.find(p.key);
      if (it == values.end()) {
        failures << "\n  " << pair.name << " " << p.key << ": no base value";
        ++count;
        continue;
      }
      std::set<double> reported;  // one line per distinct value
      for (const double x : numbers(it->second)) {
        const std::string why = violation(p, x);
        if (!why.empty() && reported.insert(x).second) {
          failures << "\n  " << pair.name << " " << p.key << " = " << x << ": " << why;
          ++count;
        }
      }
    }
  }
  EXPECT_EQ(count, 0U) << "values outside [0.1, 1000] once displayed:" << failures.str();
}

TEST(StudioCatalogDisplay, ExceptionsNameCatalogKeys) {
  for (const auto& [key, kind] : exceptions()) {
    const ParamInfo* p = find_param(key);
    ASSERT_NE(p, nullptr) << key;
    EXPECT_TRUE(is_numeric(*p)) << key;
    if (kind == Exception::kFraction) {
      EXPECT_TRUE(p->unit.empty()) << key << " is listed as a unitless fraction";
    }
  }
}

}  // namespace
}  // namespace cobot::studio

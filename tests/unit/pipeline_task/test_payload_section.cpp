// Controller section cobot.payload: {kg, offset_m, radius_m} (brief hardware-catalog 4). Absent =
// 0, keys validated like the imperfection ones. controller_model merges it into the last link with
// the function perturb_model uses (imperfect::merge_payload); 0 kg leaves the model untouched, so
// the computed-torque hashes do not move.
#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/perturb.hpp"
#include "cobot/model/model.hpp"
#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/overrides.hpp"

#include "hw_test_util.hpp"

namespace cobot::pipeline {
namespace {

using namespace hw_test;
using common::Mat3;
using common::Vec3;

constexpr double kTol = 1e-12;
const fs::path kDefault = kRepo / "config/default.yaml";

// config/default.yaml with `section` appended under cobot: and the relative paths made absolute.
fs::path default_with_section(const std::string& section) {
  std::string text = read_text(kDefault);
  text = set_line(text,
                  "  urdf_path:",
                  "  urdf_path: " + (kRepo / "src/sim/urdf/cobot30_5.urdf").generic_string());
  text = set_line(text, "    output_dir:", "    output_dir: " + test_dir().generic_string());
  if (text.back() != '\n') {
    text += '\n';
  }
  return write_text(test_dir() / "config.yaml", text + section + "\n");
}

model::Arm7 urdf_arm() {
  const auto chain =
      model::parse_urdf_chain(kRepo / "src/sim/urdf/cobot30_5.urdf", "base_link", "link_7");
  EXPECT_TRUE(chain.is_ok()) << chain.reason();
  const auto arm = model::make_model<kN>(chain.value(), Vec3(0.0, 0.0, -9.80665));
  EXPECT_TRUE(arm.is_ok()) << arm.reason();
  return arm.value();
}

void expect_links_bit_equal(const model::Arm7& a, const model::Arm7& b) {
  for (std::size_t k = 0; k < static_cast<std::size_t>(kN); ++k) {
    EXPECT_EQ(a.links[k].mass_kg, b.links[k].mass_kg) << k;
    EXPECT_TRUE(a.links[k].com_i == b.links[k].com_i) << k;
    EXPECT_TRUE(a.links[k].I_com_i == b.links[k].I_com_i) << k;
    EXPECT_EQ(a.joints[k].tau_max, b.joints[k].tau_max) << k;
  }
  EXPECT_TRUE(a.T_last_tip.p == b.T_last_tip.p);
  EXPECT_TRUE(a.T_last_tip.R == b.T_last_tip.R);
}

void expect_section_error(const std::string& section, const std::string& key) {
  SCOPED_TRACE(section);
  const auto r = load_config<kN>(default_with_section(section));
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), key)) << r.reason();
}

TEST(PayloadSection, AbsentIsZero) {
  const auto r = load_config<kN>(kDefault);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().payload.kg, 0.0);
  EXPECT_TRUE(r.value().payload.offset_m == Vec3::Zero());
  EXPECT_EQ(r.value().payload.radius_m, 0.0);
}

TEST(PayloadSection, IsParsed) {
  const auto r = load_config<kN>(
      default_with_section("  payload: {kg: 3.0, offset_m: [0.01, -0.02, 0.05], radius_m: 0.05}"));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().payload.kg, 3.0);
  EXPECT_TRUE(r.value().payload.offset_m == Vec3(0.01, -0.02, 0.05));
  EXPECT_EQ(r.value().payload.radius_m, 0.05);
}

TEST(PayloadSection, StrictKeys) {
  expect_section_error("  payload: {kg: 1.0, offset_m: [0, 0, 0], radius_m: 0.0, bogus_key: 1}",
                       "cobot.payload.bogus_key");
  expect_section_error("  payload: 5", "cobot.payload");
}

TEST(PayloadSection, ValuesAreValidatedLikeTheImperfectionOnes) {
  const std::string off = "offset_m: [0, 0, 0]";
  expect_section_error("  payload: {kg: -1.0, " + off + ", radius_m: 0.0}", "cobot.payload.kg");
  expect_section_error("  payload: {kg: .nan, " + off + ", radius_m: 0.0}", "cobot.payload.kg");
  expect_section_error("  payload: {kg: abc, " + off + ", radius_m: 0.0}", "cobot.payload.kg");
  expect_section_error("  payload: {kg: 1.0, " + off + ", radius_m: -0.01}",
                       "cobot.payload.radius_m");
  expect_section_error("  payload: {kg: 1.0, " + off + ", radius_m: .nan}",
                       "cobot.payload.radius_m");
  for (const char* offset :
       {"[0, 0]", "[0, 0, 0, 0]", "0.1", "[.nan, 0, 0]", "[0, .inf, 0]", "[a, 0, 0]"}) {
    expect_section_error(
        std::string("  payload: {kg: 1.0, offset_m: ") + offset + ", radius_m: 0.0}",
        "cobot.payload.offset_m");
  }
}

TEST(PayloadSection, UnknownKeyThroughOverridesIsTheLoaderError) {
  const auto r = load_config_with<kN>(kDefault, {KeyOverride{"config.payload.bogus_key", 1.0}});
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "cobot.payload.bogus_key")) << r.reason();
}

// --- controller_model ---

TEST(PayloadControllerModel, NoSectionLeavesTheLinksOfTheUrdf) {
  const model::Arm7 urdf = urdf_arm();
  const auto c = load_config<kN>(kDefault);
  ASSERT_TRUE(c.is_ok()) << c.reason();
  const model::Arm7 out = controller_model<kN>(urdf, c.value());
  for (std::size_t k = 0; k < static_cast<std::size_t>(kN); ++k) {
    EXPECT_EQ(out.links[k].mass_kg, urdf.links[k].mass_kg) << k;
    EXPECT_TRUE(out.links[k].com_i == urdf.links[k].com_i) << k;
    EXPECT_TRUE(out.links[k].I_com_i == urdf.links[k].I_com_i) << k;
  }
}

// 0 kg ignores offset and radius, bit for bit equal to no section.
TEST(PayloadControllerModel, ZeroKgIsBitEqualToNoSection) {
  const model::Arm7 urdf = urdf_arm();
  const auto none = load_config<kN>(kDefault);
  ASSERT_TRUE(none.is_ok()) << none.reason();
  const model::Arm7 expected = controller_model<kN>(urdf, none.value());
  const auto zero = load_config_with<kN>(
      kDefault,
      {KeyOverride{"config.payload.kg", 0.0},
       KeyOverride{"config.payload.offset_m", std::vector<double>{0.1, -0.2, 0.3}},
       KeyOverride{"config.payload.radius_m", 0.05}});
  ASSERT_TRUE(zero.is_ok()) << zero.reason();
  expect_links_bit_equal(controller_model<kN>(urdf, zero.value()), expected);
}

// Same arithmetic as the plant's: m' = m + 3, and the rest within 1e-12 of perturb_model with zero
// mismatch and the same payload. The links before the last and the peak torque stay as they were.
TEST(PayloadControllerModel, MergeEqualsPerturbModelWithZeroMismatch) {
  constexpr double kKg = 3.0;
  const Vec3 offset(0.01, -0.02, 0.05);
  constexpr double kRadius = 0.05;

  const model::Arm7 urdf = urdf_arm();
  const auto none = load_config<kN>(kDefault);
  ASSERT_TRUE(none.is_ok()) << none.reason();
  const model::Arm7 base = controller_model<kN>(urdf, none.value());
  const auto c =
      load_config_with<kN>(kDefault,
                           {KeyOverride{"config.payload.kg", kKg},
                            KeyOverride{"config.payload.offset_m",
                                        std::vector<double>{offset.x(), offset.y(), offset.z()}},
                            KeyOverride{"config.payload.radius_m", kRadius}});
  ASSERT_TRUE(c.is_ok()) << c.reason();
  const model::Arm7 out = controller_model<kN>(urdf, c.value());

  imperfect::ModelMismatchConfig<kN> mm;
  mm.payload_kg = kKg;
  mm.payload_offset_m = offset;
  mm.payload_radius_m = kRadius;
  const model::Arm7 plant = imperfect::perturb_model<kN>(urdf, mm, 1);

  constexpr std::size_t kLast = static_cast<std::size_t>(kN) - 1;
  EXPECT_NEAR(out.links[kLast].mass_kg, urdf.links[kLast].mass_kg + kKg, kTol);
  EXPECT_NEAR(out.links[kLast].mass_kg, plant.links[kLast].mass_kg, kTol);
  EXPECT_LT((out.links[kLast].com_i - plant.links[kLast].com_i).cwiseAbs().maxCoeff(), kTol);
  const Mat3 d_inertia = out.links[kLast].I_com_i - plant.links[kLast].I_com_i;
  EXPECT_LT(d_inertia.cwiseAbs().maxCoeff(), kTol) << d_inertia;
  EXPECT_GT((out.links[kLast].I_com_i - urdf.links[kLast].I_com_i).cwiseAbs().maxCoeff(), 1e-6)
      << "the payload changed the inertia";

  for (std::size_t k = 0; k < kLast; ++k) {
    EXPECT_EQ(out.links[k].mass_kg, urdf.links[k].mass_kg) << k;
    EXPECT_TRUE(out.links[k].I_com_i == urdf.links[k].I_com_i) << k;
  }
  for (std::size_t k = 0; k < static_cast<std::size_t>(kN); ++k) {
    EXPECT_EQ(out.joints[k].tau_max, base.joints[k].tau_max) << k;
  }
}

}  // namespace
}  // namespace cobot::pipeline

// ParamInfo::gazebo_supported (feature studio-control, cobot_studio design 6.5 and 6.7): false for
// what cobot_controller rejects or ignores on the Gazebo path (on_configure): the payload keys
// (rejected), friction (set in the physics at launch), transmission and sensor.position_source
// (ignored with a warning), besides the timing and comm keys (bench only). Every other catalog
// key stays supported; the Configuration tab greys the unsupported rows while connected.
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/studio/param_catalog.hpp"

namespace cobot::studio {
namespace {

bool starts_with(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

// The rule of the brief, by key.
bool unsupported_on_gazebo(const std::string& key) {
  return key == "model.payload_kg" || key == "model.payload_offset_m" ||
         key == "model.payload_radius_m" || key == "sensor.position_source" ||
         starts_with(key, "friction.") || starts_with(key, "transmission.") ||
         starts_with(key, "timing.") || starts_with(key, "comm.");
}

TEST(StudioCatalogGazebo, NamedKeysAreNotSupportedOnGazebo) {
  const std::vector<std::string> keys = {"model.payload_kg",
                                         "model.payload_offset_m",
                                         "model.payload_radius_m",
                                         "friction.coulomb_nm",
                                         "friction.viscous_nm_s_rad",
                                         "transmission.stiffness_k1_nm_rad",
                                         "transmission.stiffness_k2_nm_rad",
                                         "transmission.stiffness_k3_nm_rad",
                                         "transmission.torque_t1_nm",
                                         "transmission.torque_t2_nm",
                                         "transmission.damping_ratio",
                                         "transmission.error_amp_rad",
                                         "transmission.lost_motion_rad",
                                         "transmission.ratio",
                                         "sensor.position_source",
                                         "timing.period_jitter_std_us",
                                         "timing.overrun_prob",
                                         "timing.overrun_cycles",
                                         "comm.servo_rate_hz",
                                         "comm.packet_loss_prob",
                                         "comm.latency_burst_prob",
                                         "comm.burst_cycles"};
  for (const std::string& key : keys) {
    const ParamInfo* p = find_param(key);
    ASSERT_NE(p, nullptr) << key << " is not in the catalog";
    EXPECT_FALSE(p->gazebo_supported) << key;
  }
}

TEST(StudioCatalogGazebo, SensorNoiseStaysSupported) {
  const ParamInfo* p = find_param("sensor.q_noise_std_rad");
  ASSERT_NE(p, nullptr);
  EXPECT_TRUE(p->gazebo_supported);
}

// Exactly the keys of the rule are unsupported: the controller keys, the other sensor, actuator
// and model keys apply on Gazebo as on the bench.
TEST(StudioCatalogGazebo, EveryOtherKeyIsSupported) {
  for (const ParamInfo& p : param_catalog()) {
    EXPECT_EQ(p.gazebo_supported, !unsupported_on_gazebo(p.key)) << p.key;
  }
}

}  // namespace
}  // namespace cobot::studio

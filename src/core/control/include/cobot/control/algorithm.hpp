#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

// Closed registry of the tracking laws (ADR-023 "Thuật toán control.algorithm", feature
// controller-registry). "Registered at build time" means an entry in kAlgorithms and a case in
// the switch of TrackingController::compute (tracking_controller.cpp): no virtual function, no
// std::function, no heap. The YAML key cobot.control.algorithm takes the table's names.
namespace cobot::control {

// Control algorithms registered at build time (ADR-023). Values are stable: append only.
enum class Algorithm : std::uint8_t { kComputedTorque = 0, kPdGravity = 1, kDrivePosition = 2 };

// Torque feed-forward the host sends with a drive_position set point (YAML
// cobot.drive.torque_feedforward: none | gravity | inverse_dynamics; DrivePosition).
enum class DriveFeedForward : std::uint8_t { kNone, kGravity, kInverseDynamics };

struct AlgorithmInfo {
  Algorithm id;
  const char* name;     // YAML value of cobot.control.algorithm
  const char* summary;  // one line, for reports and the studio choice list
};

// Entry i has id Algorithm(i).
inline constexpr std::array<AlgorithmInfo, 3> kAlgorithms{{
    {Algorithm::kComputedTorque,
     "computed_torque",
     "joint-space computed torque (inverse dynamics)"},
    {Algorithm::kPdGravity, "pd_gravity", "joint PD plus gravity compensation"},
    {Algorithm::kDrivePosition,
     "drive_position",
     "position loop in the joint drive, set points from the host"},
}};

// The table's name; "unknown" for a value outside the table.
const char* to_string(Algorithm a);
// Exact, case-sensitive match against the table's names.
std::optional<Algorithm> algorithm_from_string(std::string_view s);
// "computed_torque, pd_gravity, drive_position" (allocates: config time only).
std::string registered_algorithm_names();

}  // namespace cobot::control

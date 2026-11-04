#pragma once

#include <array>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/model/model.hpp"

// The arm as the 3D view draws it (feature studio-3d, cobot_studio design 6.6, ADR-021): the
// <visual> of every link on the base -> tip chain, placed by forward_kinematics on the kinematic
// model of the same URDF. No Qt.
namespace cobot::studio {

struct VisualLink {
  std::string link;
  std::filesystem::path mesh;                         // resolved file path
  common::Pose T_frame_visual;                        // <visual><origin> in the link frame
  std::array<float, 4> rgba{0.8F, 0.8F, 0.8F, 1.0F};  // <material><color>
  int joint = -1;             // revolute joint whose frame carries this link; -1: base
  common::Pose T_joint_link;  // fixed offset from that frame (fixed joints folded in)
};

class RobotScene {
 public:
  // package://<pkg>/<rest> resolved through `packages` (cobot_sim -> <repo>/src/sim); other mesh
  // file names are relative to the URDF's directory. Errors name the link and the URI or path.
  // Builds the kinematic model from the same URDF (model::make_model). Links with a <visual> that
  // are not carried by base_link or a chain joint (side branches) are left out.
  static common::Result<RobotScene> load(
      const std::filesystem::path& urdf,
      const std::string& base_link,
      const std::string& tip_link,
      const std::map<std::string, std::filesystem::path>& packages);
  const std::vector<VisualLink>& links() const { return links_; }
  // World pose of every link's visual for joint angles q; `out` sized once by the caller
  // (links().size()). No heap allocation.
  void visual_poses(const common::VecN<model::kArmDof>& q, std::vector<common::Pose>& out) const;
  common::Pose tip_pose(const common::VecN<model::kArmDof>& q) const;  // == FK T_b_e

 private:
  std::vector<VisualLink> links_;
  model::Arm7 model_;
};

}  // namespace cobot::studio

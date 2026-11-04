#include "cobot/studio/robot_scene.hpp"

#include <cstddef>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>

#include <Eigen/Geometry>

#include "cobot/kinematics/kinematics.hpp"

#include <tinyxml2.h>

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using tinyxml2::XMLElement;

struct JointInfo {
  std::string parent, type;
  common::Pose origin;
};

common::Vec3 read_vec3(const XMLElement* e, const char* attribute) {
  common::Vec3 v = common::Vec3::Zero();
  const char* text = e == nullptr ? nullptr : e->Attribute(attribute);
  if (text != nullptr) {
    std::istringstream(text) >> v.x() >> v.y() >> v.z();
  }
  return v;
}

// <origin xyz rpy> child of `parent`, URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll); identity when
// absent. The chain's own origins were validated by model::parse_urdf_chain.
common::Pose read_origin(const XMLElement* parent) {
  const XMLElement* origin = parent->FirstChildElement("origin");
  const common::Vec3 rpy = read_vec3(origin, "rpy");
  common::Pose T;
  T.R = (Eigen::AngleAxisd(rpy.z(), common::Vec3::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), common::Vec3::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), common::Vec3::UnitX()))
            .toRotationMatrix();
  T.p = read_vec3(origin, "xyz");
  return T;
}

// <material> colour: inline <color rgba>, else the named top-level material, else the default.
void read_rgba(const XMLElement* visual, const XMLElement* robot, std::array<float, 4>& rgba) {
  const XMLElement* material = visual->FirstChildElement("material");
  if (material == nullptr) {
    return;
  }
  const XMLElement* color = material->FirstChildElement("color");
  const char* name = material->Attribute("name");
  for (const XMLElement* m = robot->FirstChildElement("material"); color == nullptr && m != nullptr;
       m = m->NextSiblingElement("material")) {
    if (name != nullptr && m->Attribute("name", name) != nullptr) {
      color = m->FirstChildElement("color");
    }
  }
  const char* text = color == nullptr ? nullptr : color->Attribute("rgba");
  if (text != nullptr) {
    std::istringstream(text) >> rgba[0] >> rgba[1] >> rgba[2] >> rgba[3];
  }
}

common::Result<fs::path> resolve(const std::string& uri,
                                 const fs::path& urdf_dir,
                                 const std::map<std::string, fs::path>& packages,
                                 const std::string& link) {
  static constexpr std::string_view kPackage = "package://";
  fs::path file = urdf_dir / uri;
  if (uri.compare(0, kPackage.size(), kPackage) == 0) {
    const std::string rest = uri.substr(kPackage.size());
    const std::size_t slash = rest.find('/');
    const auto pkg = packages.find(rest.substr(0, slash));
    if (slash == std::string::npos || pkg == packages.end()) {
      return common::Result<fs::path>::error(link + ": unknown package in mesh '" + uri + "'");
    }
    file = pkg->second / rest.substr(slash + 1);
  }
  std::error_code ec;
  if (!fs::is_regular_file(file, ec)) {
    return common::Result<fs::path>::error(link + ": mesh file not found: " + file.string());
  }
  return common::Result<fs::path>::ok(file);
}

}  // namespace

common::Result<RobotScene> RobotScene::load(const fs::path& urdf,
                                            const std::string& base_link,
                                            const std::string& tip_link,
                                            const std::map<std::string, fs::path>& packages) {
  using R = common::Result<RobotScene>;
  const auto chain = model::parse_urdf_chain(urdf, base_link, tip_link);
  if (!chain) {
    return R::error(urdf.string() + ": " + chain.reason());
  }
  auto arm = model::make_model<model::kArmDof>(chain.value(), common::Vec3(0.0, 0.0, -9.80665));
  if (!arm) {
    return R::error(urdf.string() + ": " + arm.reason());
  }
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(urdf.string().c_str()) != tinyxml2::XML_SUCCESS) {
    return R::error(urdf.string() + ": cannot read the URDF");
  }
  const XMLElement* robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    return R::error(urdf.string() + ": no <robot> element");
  }
  std::map<std::string, JointInfo> joint_of_child;  // child link -> joint
  std::map<std::string, int> chain_index;           // revolute chain joint's child link -> i
  for (const XMLElement* j = robot->FirstChildElement("joint"); j != nullptr;
       j = j->NextSiblingElement("joint")) {
    const XMLElement* parent = j->FirstChildElement("parent");
    const XMLElement* child = j->FirstChildElement("child");
    if (parent == nullptr || child == nullptr || child->Attribute("link") == nullptr) {
      continue;
    }
    const std::string child_link = child->Attribute("link");
    joint_of_child[child_link] = {
        parent->Attribute("link") == nullptr ? "" : parent->Attribute("link"),
        j->Attribute("type") == nullptr ? "" : j->Attribute("type"),
        read_origin(j)};
    for (int i = 0; i < model::kArmDof; ++i) {
      if (arm.value().joints[static_cast<std::size_t>(i)].name == j->Attribute("name")) {
        chain_index[child_link] = i;
      }
    }
  }

  RobotScene scene;
  scene.model_ = arm.value();
  for (const XMLElement* l = robot->FirstChildElement("link"); l != nullptr;
       l = l->NextSiblingElement("link")) {
    const XMLElement* visual = l->FirstChildElement("visual");
    const XMLElement* geometry =
        visual == nullptr ? nullptr : visual->FirstChildElement("geometry");
    const XMLElement* mesh = geometry == nullptr ? nullptr : geometry->FirstChildElement("mesh");
    if (mesh == nullptr || l->Attribute("name") == nullptr) {
      continue;
    }
    VisualLink v;
    v.link = l->Attribute("name");
    // Walk up through fixed joints to base_link or the revolute chain joint that carries it.
    bool carried = false;
    for (std::string cur = v.link;;) {
      if (cur == base_link) {
        carried = true;
        break;
      }
      const auto c = chain_index.find(cur);
      if (c != chain_index.end()) {
        v.joint = c->second;
        carried = true;
        break;
      }
      const auto j = joint_of_child.find(cur);
      if (j == joint_of_child.end() || j->second.type != "fixed") {
        break;
      }
      v.T_joint_link = common::compose(j->second.origin, v.T_joint_link);
      cur = j->second.parent;
    }
    if (!carried) {
      continue;
    }
    const char* filename = mesh->Attribute("filename");
    auto file = resolve(filename == nullptr ? "" : filename, urdf.parent_path(), packages, v.link);
    if (!file) {
      return R::error(file.reason());
    }
    v.mesh = std::move(file.value());
    v.T_frame_visual = read_origin(visual);
    read_rgba(visual, robot, v.rgba);
    scene.links_.push_back(std::move(v));
  }
  return R::ok(std::move(scene));
}

void RobotScene::visual_poses(const common::VecN<model::kArmDof>& q,
                              std::vector<common::Pose>& out) const {
  kinematics::FkState<model::kArmDof> fk;
  kinematics::forward_kinematics(model_, q, fk);
  for (std::size_t i = 0; i < links_.size() && i < out.size(); ++i) {
    const VisualLink& v = links_[i];
    const common::Pose& T_b_joint =
        v.joint < 0 ? common::Pose{} : fk.T_b_i[static_cast<std::size_t>(v.joint)];
    out[i] = common::compose(common::compose(T_b_joint, v.T_joint_link), v.T_frame_visual);
  }
}

common::Pose RobotScene::tip_pose(const common::VecN<model::kArmDof>& q) const {
  kinematics::FkState<model::kArmDof> fk;
  kinematics::forward_kinematics(model_, q, fk);
  return fk.T_b_e;
}

}  // namespace cobot::studio

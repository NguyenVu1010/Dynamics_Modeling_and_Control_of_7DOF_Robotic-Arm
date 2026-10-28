#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "cobot/model/model.hpp"

#include <tinyxml2.h>

// URDF subset reader (ADR-005, docs/conventions.md "Model and fixed-joint merge"). Runs once
// at start-up, so it may allocate; it reports problems through Result and never throws.
namespace cobot::model {
namespace {

using common::Status;
using tinyxml2::XMLElement;

constexpr double kMinAxisNorm = 1e-12;  // below this an <axis xyz> counts as zero

// Concatenates the parts of an error message.
std::string cat(std::initializer_list<std::string_view> parts) {
  std::string out;
  for (const std::string_view part : parts) {
    out.append(part);
  }
  return out;
}

std::string attr_name(const char* element, const char* attribute) {
  return std::string("<") + element + " " + attribute + ">";
}

// Three whitespace-separated finite numbers and nothing else.
bool parse_vec3(const char* text, Vec3& out) {
  const char* cursor = text;
  for (int k = 0; k < 3; ++k) {
    char* end = nullptr;
    const double value = std::strtod(cursor, &end);
    if (end == cursor || !std::isfinite(value)) {
      return false;
    }
    out[k] = value;
    cursor = end;
  }
  for (; *cursor != '\0'; ++cursor) {
    if (std::isspace(static_cast<unsigned char>(*cursor)) == 0) {
      return false;
    }
  }
  return true;
}

// Vector attribute of `element`; `fallback` when the element or the attribute is absent.
Status read_vec3(const XMLElement* element,
                 const char* element_name,
                 const char* attribute,
                 const std::string& owner,
                 const Vec3& fallback,
                 Vec3& out) {
  const char* text = element != nullptr ? element->Attribute(attribute) : nullptr;
  if (text == nullptr) {
    out = fallback;
    return Status::ok();
  }
  if (!parse_vec3(text, out)) {
    return Status::error(owner + ": " + attr_name(element_name, attribute) +
                         " must hold 3 numbers, got '" + text + "'");
  }
  return Status::ok();
}

// Scalar attribute of `element`. Absent element or attribute: `*fallback`, or an error
// when `fallback` is null.
Status read_double(const XMLElement* element,
                   const char* element_name,
                   const char* attribute,
                   const std::string& owner,
                   const double* fallback,
                   double& out) {
  const char* text = element != nullptr ? element->Attribute(attribute) : nullptr;
  if (text == nullptr) {
    if (fallback == nullptr) {
      return Status::error(owner + ": " + attr_name(element_name, attribute) + " missing");
    }
    out = *fallback;
    return Status::ok();
  }
  if (element->QueryDoubleAttribute(attribute, &out) != tinyxml2::XML_SUCCESS ||
      !std::isfinite(out)) {
    return Status::error(owner + ": " + attr_name(element_name, attribute) + " is not a number: '" +
                         text + "'");
  }
  return Status::ok();
}

// URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll).
Mat3 rpy_to_rotation(const Vec3& rpy) {
  const Mat3 R =
      (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
       Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()))
          .toRotationMatrix();
  return R;
}

// <origin xyz rpy> child of `parent`; identity when absent.
Status read_origin(const XMLElement* parent, const std::string& owner, Pose& out) {
  const XMLElement* origin = parent->FirstChildElement("origin");
  Vec3 rpy = Vec3::Zero();
  Status s = read_vec3(origin, "origin", "xyz", owner, Vec3::Zero(), out.p);
  if (s.is_ok()) {
    s = read_vec3(origin, "origin", "rpy", owner, Vec3::Zero(), rpy);
  }
  out.R = rpy_to_rotation(rpy);
  return s;
}

// <inertial> of a link, expressed in the link frame. No <inertial>: massless link.
Status read_inertial(const XMLElement* link, const std::string& name, LinkInertia& out) {
  out = LinkInertia{};
  const XMLElement* inertial = link->FirstChildElement("inertial");
  if (inertial == nullptr) {
    return Status::ok();
  }
  Pose T_link_inertial;
  Status s = read_origin(inertial, name, T_link_inertial);
  if (!s.is_ok()) {
    return s;
  }
  s = read_double(inertial->FirstChildElement("mass"), "mass", "value", name, nullptr, out.mass_kg);
  if (!s.is_ok()) {
    return s;
  }
  if (out.mass_kg < 0.0) {
    return Status::error(name + ": <mass value> is negative");
  }
  const XMLElement* tensor = inertial->FirstChildElement("inertia");
  constexpr int kTerms = 6;
  const char* const names[kTerms] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"};
  double v[kTerms] = {};
  for (int k = 0; k < kTerms; ++k) {
    s = read_double(tensor, "inertia", names[k], name, nullptr, v[k]);
    if (!s.is_ok()) {
      return s;
    }
  }
  Mat3 I_inertial;
  I_inertial << v[0], v[1], v[2], v[1], v[3], v[4], v[2], v[4], v[5];
  out.com_i = T_link_inertial.p;
  out.I_com_i = T_link_inertial.R * I_inertial * T_link_inertial.R.transpose();
  return Status::ok();
}

// Parallel-axis term for mass m whose centre of mass sits at d from the reference point.
Mat3 steiner(double m, const Vec3& d) {
  const Mat3 S = m * (d.squaredNorm() * Mat3::Identity() - d * d.transpose());
  return S;
}

// Adds `child` (expressed in its own frame) to `acc` (frame a) through T_a_child.
void merge_into(LinkInertia& acc, const LinkInertia& child, const Pose& T_a_child) {
  const Vec3 c_child = common::transform(T_a_child, child.com_i);
  const Mat3 I_child = T_a_child.R * child.I_com_i * T_a_child.R.transpose();
  const double m = acc.mass_kg + child.mass_kg;
  if (m <= 0.0) {
    acc.I_com_i += I_child;
    return;
  }
  const Vec3 c = (acc.mass_kg * acc.com_i + child.mass_kg * c_child) / m;
  const Mat3 I = acc.I_com_i + steiner(acc.mass_kg, acc.com_i - c) + I_child +
                 steiner(child.mass_kg, c_child - c);
  acc.mass_kg = m;
  acc.com_i = c;
  acc.I_com_i = I;
}

// Revolute or continuous joint; T_prefix is the fixed-joint transform composed in front of
// its <origin>.
Status read_revolute(const XMLElement* joint,
                     const std::string& name,
                     bool continuous,
                     const Pose& T_prefix,
                     JointParams& out) {
  out.name = name;
  Pose T_origin;
  Status s = read_origin(joint, name, T_origin);
  if (!s.is_ok()) {
    return s;
  }
  out.T_parent_joint = common::compose(T_prefix, T_origin);

  Vec3 axis = Vec3::UnitX();  // URDF default
  s = read_vec3(joint->FirstChildElement("axis"), "axis", "xyz", name, Vec3::UnitX(), axis);
  if (!s.is_ok()) {
    return s;
  }
  if (axis.norm() < kMinAxisNorm) {
    return Status::error(name + ": <axis xyz> is zero");
  }
  out.axis_joint = axis.normalized();

  const XMLElement* limit = joint->FirstChildElement("limit");
  if (limit == nullptr) {
    return Status::error(name + ": <limit> missing");
  }
  s = read_double(limit, "limit", "effort", name, nullptr, out.tau_max);
  if (s.is_ok()) {
    s = read_double(limit, "limit", "velocity", name, nullptr, out.qd_max);
  }
  if (!s.is_ok()) {
    return s;
  }
  if (continuous) {
    out.q_min = -std::numeric_limits<double>::infinity();
    out.q_max = std::numeric_limits<double>::infinity();
  } else {
    s = read_double(limit, "limit", "lower", name, nullptr, out.q_min);
    if (s.is_ok()) {
      s = read_double(limit, "limit", "upper", name, nullptr, out.q_max);
    }
    if (!s.is_ok()) {
      return s;
    }
    if (out.q_min > out.q_max) {
      return Status::error(name + ": <limit lower> is above <limit upper>");
    }
  }
  constexpr double kNoDamping = 0.0;
  return read_double(
      joint->FirstChildElement("dynamics"), "dynamics", "damping", name, &kNoDamping, out.damping);
}

// `link` attribute of the joint's <parent> or <child>; null when absent.
const char* joint_link(const XMLElement* joint, const char* tag) {
  const XMLElement* e = joint->FirstChildElement(tag);
  return e != nullptr ? e->Attribute("link") : nullptr;
}

struct UrdfGraph {
  std::map<std::string, const XMLElement*> links;
  std::map<std::string, const XMLElement*> joint_by_child;  // child link -> joint
};

Status index_robot(const XMLElement* robot, UrdfGraph& g) {
  for (const XMLElement* link = robot->FirstChildElement("link"); link != nullptr;
       link = link->NextSiblingElement("link")) {
    const char* name = link->Attribute("name");
    if (name == nullptr) {
      return Status::error("<link name> missing");
    }
    if (!g.links.emplace(name, link).second) {
      return Status::error(std::string("link '") + name + "' defined twice");
    }
  }
  for (const XMLElement* joint = robot->FirstChildElement("joint"); joint != nullptr;
       joint = joint->NextSiblingElement("joint")) {
    const char* name = joint->Attribute("name");
    if (name == nullptr) {
      return Status::error("<joint name> missing");
    }
    const char* parent = joint_link(joint, "parent");
    const char* child = joint_link(joint, "child");
    if (parent == nullptr || child == nullptr) {
      return Status::error(std::string(name) + ": <parent link> or <child link> missing");
    }
    if (!g.joint_by_child.emplace(child, joint).second) {
      return Status::error(std::string(name) + ": link '" + child + "' already has a parent");
    }
  }
  return Status::ok();
}

// Joints from base_link to tip_link, base first.
Status find_path(const UrdfGraph& g,
                 const std::string& base,
                 const std::string& tip,
                 std::vector<const XMLElement*>& path) {
  for (const std::string& link : {base, tip}) {
    if (g.links.count(link) == 0) {
      return Status::error("link '" + link + "' not found");
    }
  }
  std::string current = tip;
  while (current != base) {
    const auto it = g.joint_by_child.find(current);
    if (it == g.joint_by_child.end() || path.size() > g.joint_by_child.size()) {
      return Status::error(cat({"no chain from '", base, "' to '", tip, "'"}));
    }
    path.push_back(it->second);
    current = joint_link(it->second, "parent");
  }
  std::reverse(path.begin(), path.end());
  return Status::ok();
}

Status build_chain(const UrdfGraph& g,
                   const std::vector<const XMLElement*>& path,
                   UrdfChain& chain) {
  Pose T_prefix;  // frame of the last revolute joint's child (or base_link) -> current link
  for (const XMLElement* joint : path) {
    const std::string name = joint->Attribute("name");
    const char* type_attr = joint->Attribute("type");
    const std::string type = type_attr != nullptr ? type_attr : "";
    const std::string child = joint_link(joint, "child");
    const auto link = g.links.find(child);
    if (link == g.links.end()) {
      return Status::error(cat({name, ": child link '", child, "' not found"}));
    }
    LinkInertia inertia;
    Status s = read_inertial(link->second, child, inertia);
    if (!s.is_ok()) {
      return s;
    }
    if (type == "fixed") {
      Pose T_origin;
      s = read_origin(joint, name, T_origin);
      if (!s.is_ok()) {
        return s;
      }
      T_prefix = common::compose(T_prefix, T_origin);
      if (!chain.links.empty()) {  // links before the first revolute joint belong to the base
        merge_into(chain.links.back(), inertia, T_prefix);
      }
    } else if (type == "revolute" || type == "continuous") {
      JointParams params;
      s = read_revolute(joint, name, type == "continuous", T_prefix, params);
      if (!s.is_ok()) {
        return s;
      }
      chain.joints.push_back(std::move(params));
      chain.links.push_back(inertia);
      T_prefix = Pose{};
    } else {
      return Status::error(cat(
          {name, ": joint type '", type, "' is not supported (revolute, continuous or fixed)"}));
    }
  }
  chain.T_last_tip = T_prefix;
  return Status::ok();
}

}  // namespace

Result<UrdfChain> parse_urdf_chain(const std::filesystem::path& urdf,
                                   const std::string& base_link,
                                   const std::string& tip_link) {
  const std::string file = urdf.string();
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS) {
    return Result<UrdfChain>::error(file + ": " + doc.ErrorStr());
  }
  const XMLElement* robot = doc.RootElement();
  if (robot == nullptr || std::string(robot->Name()) != "robot") {
    return Result<UrdfChain>::error(file + ": root element is not <robot>");
  }
  UrdfGraph graph;
  std::vector<const XMLElement*> path;
  UrdfChain chain;
  Status s = index_robot(robot, graph);
  if (s.is_ok()) {
    s = find_path(graph, base_link, tip_link, path);
  }
  if (s.is_ok()) {
    s = build_chain(graph, path, chain);
  }
  if (!s.is_ok()) {
    return Result<UrdfChain>::error(file + ": " + s.reason());
  }
  return Result<UrdfChain>::ok(std::move(chain));
}

}  // namespace cobot::model

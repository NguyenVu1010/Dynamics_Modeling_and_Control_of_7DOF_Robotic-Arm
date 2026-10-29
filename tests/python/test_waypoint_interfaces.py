"""ROS interfaces and wiring of the external command mode (feature waypoint-ros), checked from the
files (no ROS needed).

- New colcon package `cobot_interfaces` in ros2_interfaces/ with the message, action and service
  fields of the brief (order free, names and types exact).
- sim.sh builds it with the other colcon packages (so the gazebo CI job does too).
- sim.launch.py has a `command_source` argument (default "") that reaches cobot_controller as a
  parameter only when set.
- cobot_ros2 depends on it and installs tools/send_waypoints.py as an executable of the package
  (run_e2e calls `ros2 run cobot_ros2 send_waypoints.py`).
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from test_sim_launch_realism import run_launch  # noqa: F401  (fixture)

REPO = Path(__file__).resolve().parents[2]
PKG = REPO / "ros2_interfaces"
PKG_NAME = "cobot_interfaces"

WAYPOINT = {"pose": "geometry_msgs/Pose", "duration_s": "float64", "stop": "bool"}
WAYPOINT_ISSUE = {
    "waypoint": "uint32",
    "issue": "string",
    "joint": "int32",
    "value": "float64",
    "limit": "float64",
}
FOLLOW_GOAL = {"waypoints": "Waypoint[]"}
FOLLOW_RESULT = {
    "success": "bool",
    "message": "string",
    "issues": "WaypointIssue[]",
    "final_pos_err_m": "float64",
    "final_rot_err_deg": "float64",
    "rms_joint_err_rad": "float64",
    "peak_tau_ratio": "float64",
}
FOLLOW_FEEDBACK = {
    "segment": "uint32",
    "t_s": "float64",
    "duration_s": "float64",
    "pos_err_m": "float64",
    "rot_err_deg": "float64",
    "mode": "string",
}
PREVIEW_REQUEST = {"waypoints": "Waypoint[]"}
PREVIEW_RESPONSE = {
    "ok": "bool",
    "issues": "WaypointIssue[]",
    "path": "nav_msgs/Path",
    "segment_durations_s": "float64[]",
}


def normalize(type_name):
    """`pkg/msg/T` -> `pkg/T`; the own-package prefix is dropped (`cobot_interfaces/X` = `X`)."""
    type_name = type_name.replace("/msg/", "/")
    prefix = f"{PKG_NAME}/"
    return type_name[len(prefix) :] if type_name.startswith(prefix) else type_name


def sections(path):
    """Fields {name: type} of each `---`-separated section of a .msg/.srv/.action file."""
    assert path.is_file(), f"{path} missing"
    out = [{}]
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line == "---":
            out.append({})
            continue
        parts = line.split()
        assert len(parts) >= 2, f"{path}: cannot parse {raw!r}"
        type_name, name = parts[0], parts[1]
        if "=" in name:  # constant
            continue
        assert name not in out[-1], f"{path}: field {name} twice"
        out[-1][name] = normalize(type_name)
    return out


def test_waypoint_msg():
    assert sections(PKG / "msg/Waypoint.msg") == [WAYPOINT]


def test_waypoint_issue_msg():
    assert sections(PKG / "msg/WaypointIssue.msg") == [WAYPOINT_ISSUE]


def test_follow_waypoints_action():
    goal, result, feedback = sections(PKG / "action/FollowWaypoints.action")
    assert goal == FOLLOW_GOAL
    assert result == FOLLOW_RESULT
    assert feedback == FOLLOW_FEEDBACK


def test_preview_waypoints_srv():
    request, response = sections(PKG / "srv/PreviewWaypoints.srv")
    assert request == PREVIEW_REQUEST
    assert response == PREVIEW_RESPONSE


def package_xml(path):
    root = ET.parse(path).getroot()
    deps = {
        e.tag: {x.text.strip() for x in root.findall(e.tag)}
        for e in root
        if e.tag.endswith("depend") or e.tag == "member_of_group"
    }
    return root.findtext("name"), deps


def all_deps(deps, *tags):
    return set().union(*(deps.get(t, set()) for t in tags))


def test_interfaces_package_xml():
    name, deps = package_xml(PKG / "package.xml")
    assert name == PKG_NAME
    assert "rosidl_interface_packages" in deps.get("member_of_group", set())
    assert "rosidl_default_generators" in all_deps(deps, "buildtool_depend")
    runtime = all_deps(deps, "depend", "exec_depend", "build_depend")
    assert {"geometry_msgs", "nav_msgs"} <= runtime
    assert "rosidl_default_runtime" in all_deps(deps, "exec_depend", "depend")


def test_interfaces_cmake_generates_all_four():
    text = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
    assert re.search(rf"project\(\s*{PKG_NAME}\b", text)
    match = re.search(r"rosidl_generate_interfaces\((.*?)\)", text, re.S)
    assert match, "rosidl_generate_interfaces missing"
    body = match.group(1)
    for f in (
        "msg/Waypoint.msg",
        "msg/WaypointIssue.msg",
        "action/FollowWaypoints.action",
        "srv/PreviewWaypoints.srv",
    ):
        assert f'"{f}"' in body or f" {f}" in body or f"\n{f}" in body, f"{f} not generated"
    assert "geometry_msgs" in body and "nav_msgs" in body


def test_sim_sh_builds_the_interfaces_package():
    text = (REPO / "tools/wsl/sim.sh").read_text(encoding="utf-8")
    assert "ros2_interfaces" in text


def test_controller_package_depends_on_interfaces():
    _, deps = package_xml(REPO / "ros2_node/package.xml")
    runtime = all_deps(deps, "depend", "build_depend")
    assert {PKG_NAME, "rclcpp_action", "std_srvs", "nav_msgs"} <= runtime
    cmake = (REPO / "ros2_node/CMakeLists.txt").read_text(encoding="utf-8")
    assert re.search(rf"find_package\(\s*{PKG_NAME}\s+REQUIRED", cmake)
    assert re.search(r"find_package\(\s*rclcpp_action\s+REQUIRED", cmake)


def test_controller_installs_send_waypoints():
    cmake = (REPO / "ros2_node/CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"install\(\s*PROGRAMS([^)]*send_waypoints\.py[^)]*)\)", cmake, re.S)
    assert match, "tools/send_waypoints.py not installed with install(PROGRAMS ...)"
    assert re.search(r"DESTINATION\s+lib/(\$\{PROJECT_NAME\}|cobot_ros2)\b", match.group(1))
    assert (REPO / "tools/send_waypoints.py").is_file()


def test_controller_declares_external_endpoints():
    src = "\n".join(
        p.read_text(encoding="utf-8") for p in sorted((REPO / "ros2_node/src").glob("*.[ch]pp"))
    )
    assert re.search(r'auto_declare<std::string>\(\s*"command_source"', src)
    for name in ("~/follow_waypoints", "~/preview_waypoints", "~/reset", "~/reference_path"):
        assert f'"{name}"' in src, f"{name} not created by the controller"
    assert "RealtimeBuffer" in src


# ---------------------------------------------------------------- launch argument


def test_command_source_argument_defaults_to_empty(run_launch):  # noqa: F811
    launched = run_launch()
    assert launched.declared.get("command_source") == ""
    assert "command_source" not in launched.controller_params()


@pytest.mark.parametrize("source", ["external", "config"])
def test_command_source_reaches_controller(run_launch, source):  # noqa: F811
    params = run_launch(command_source=source).controller_params()
    assert params["command_source"] == source

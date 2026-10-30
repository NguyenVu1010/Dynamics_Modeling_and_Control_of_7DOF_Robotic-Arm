"""Gazebo Harmonic + gz_ros2_control for the 7-DOF arm.

ros2 launch cobot_sim sim.launch.py [gui:=false] [controller:=effort_passthrough]
    [config_path:=YAML] [imperfections_path:=YAML] [seed:=INT] [log_path:=CSV]
    [command_source:=config|external] [rviz:=false]

config_path, imperfections_path, seed, log_path and command_source, when given, become
parameters of the spawned controller (cobot_controller) through a temporary parameter file,
removed at shutdown; unset, the controller's defaults apply (command_source: the config file's
cobot.command_source). imperfections_path defaults to the realistic profile
(config/imperfections/realistic.yaml); seed defaults to the profile's own seed.

The profile's friction (imperfections.friction: coulomb_nm, viscous_nm_s_rad, scalar or one
value per joint) goes into the physics: it becomes the xacro args joint_<i>_friction and
joint_<i>_damping, i.e. each arm joint's <dynamics friction damping>, and the controller leaves
friction out of its plant effects on this path (ADR-011).

rviz:=true also starts RViz2 with sim/rviz/waypoints.rviz (the cobot_gui/WaypointPanel docked,
interactive markers, preview and reference paths) and runs the controller with command_source
external unless command_source is given.
"""

import json
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_JOINTS = 7


def _realistic_profile() -> str:
    share = Path(get_package_share_directory("cobot_ros2"))
    return str(share / "config/imperfections/realistic.yaml")


def _per_joint(value: object, key: str) -> list[float]:
    """A scalar applies to every joint; a list sets each joint (imperfection schema)."""
    values = [float(v) for v in value] if isinstance(value, list) else [float(value)] * ARM_JOINTS
    if len(values) != ARM_JOINTS:
        raise ValueError(f"imperfections.friction.{key}: expected {ARM_JOINTS} values")
    return values


def _friction_mappings(profile: Path) -> dict[str, str]:
    """xacro args joint_<i>_friction / joint_<i>_damping from the profile's friction."""
    import yaml  # noqa: PLC0415  (only needed once the launch runs)

    data = yaml.safe_load(profile.read_text(encoding="utf-8")) or {}
    friction = (data.get("imperfections") or {}).get("friction") or {}
    coulomb = _per_joint(friction.get("coulomb_nm", 0.0), "coulomb_nm")
    viscous = _per_joint(friction.get("viscous_nm_s_rad", 0.0), "viscous_nm_s_rad")
    mappings = {}
    for i in range(ARM_JOINTS):
        mappings[f"joint_{i + 1}_friction"] = repr(coulomb[i])
        mappings[f"joint_{i + 1}_damping"] = repr(viscous[i])
    return mappings


def _setup(context: object) -> list:
    share = Path(get_package_share_directory("cobot_sim"))
    gui = LaunchConfiguration("gui").perform(context) == "true"
    controller = LaunchConfiguration("controller").perform(context)
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context) or str(
        share / "config/controllers.yaml"
    )
    xacro_file = share / "urdf/cobot30_5.urdf.xacro"

    import xacro  # noqa: PLC0415  (only available in a sourced ROS environment)

    profile = LaunchConfiguration("imperfections_path").perform(context) or _realistic_profile()
    mappings = {"controllers_yaml": controllers_yaml, **_friction_mappings(Path(profile))}
    robot_description = xacro.process_file(str(xacro_file), mappings=mappings).toxml()

    overrides: dict[str, object] = {
        name: value
        for name in ("config_path", "imperfections_path", "log_path", "command_source")
        if (value := LaunchConfiguration(name).perform(context))
    }
    if seed := LaunchConfiguration("seed").perform(context):
        overrides["seed"] = int(seed)
    rviz = LaunchConfiguration("rviz").perform(context) == "true"
    if rviz:
        overrides.setdefault("command_source", "external")  # the panel drives the arm
    param_args = []
    cleanup = []
    if overrides:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            json.dump({controller: {"ros__parameters": overrides}}, f)  # JSON is valid YAML
        param_file = Path(f.name)
        param_args = ["--param-file", str(param_file)]
        cleanup = [
            RegisterEventHandler(
                OnShutdown(on_shutdown=lambda _event, _context: param_file.unlink(missing_ok=True))
            )
        ]

    rviz_nodes = []
    if rviz:
        rviz_nodes = [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "rviz/waypoints.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ]

    world = str(share / "worlds/empty.sdf")
    gz_args = ["gz", "sim", "-r", world] if gui else ["gz", "sim", "-r", "-s", world]
    return [
        *cleanup,
        ExecuteProcess(cmd=gz_args, output="screen"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description", "-name", "cobot30_5", "-z", "0.0"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, *param_args, "--controller-manager-timeout", "60"],
            output="screen",
        ),
        *rviz_nodes,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("controller", default_value="effort_passthrough"),
            DeclareLaunchArgument("controllers_yaml", default_value=""),
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("imperfections_path", default_value=_realistic_profile()),
            DeclareLaunchArgument("seed", default_value=""),
            DeclareLaunchArgument("log_path", default_value=""),
            DeclareLaunchArgument("command_source", default_value=""),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )

"""Gazebo Harmonic + gz_ros2_control for the 7-DOF arm.

ros2 launch cobot_sim sim.launch.py [gui:=false] [controller:=effort_passthrough]
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context: object) -> list:
    share = Path(get_package_share_directory("cobot_sim"))
    gui = LaunchConfiguration("gui").perform(context) == "true"
    controller = LaunchConfiguration("controller").perform(context)
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context) or str(
        share / "config/controllers.yaml"
    )
    xacro_file = share / "urdf/cobot30_5.urdf.xacro"

    import xacro  # noqa: PLC0415  (only available in a sourced ROS environment)

    robot_description = xacro.process_file(
        str(xacro_file), mappings={"controllers_yaml": controllers_yaml}
    ).toxml()

    world = str(share / "worlds/empty.sdf")
    gz_args = ["gz", "sim", "-r", world] if gui else ["gz", "sim", "-r", "-s", world]
    return [
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
            arguments=[controller, "--controller-manager-timeout", "60"],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("controller", default_value="effort_passthrough"),
            DeclareLaunchArgument("controllers_yaml", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simbiosys_bringup.launch_utils import (
    optional_node,
    optional_python_launch,
    package_available,
)


def generate_launch_description():
    """Launch mapping mode with existing MIRTE topics and slam_toolbox if present."""
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = (
        get_package_share_directory("simbiosys_mapping")
        + "/config/slam_toolbox_mapping.yaml"
    )

    actions = [
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/mirte_base_controller/cmd_vel",
            description="Velocity topic exposed by the MIRTE base controller.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/mirte_base_controller/odom",
            description="Odometry topic published by the MIRTE base controller.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="Laser scan topic used for mapping.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock for Gazebo mapping runs.",
        ),
        Node(
            package="simbiosys_behavior",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
        ),
        Node(
            package="simbiosys_mapping",
            executable="mapping_status_node",
            name="mapping_status_node",
            output="screen",
            parameters=[
                {
                    "scan_topic": scan_topic,
                    "odom_topic": odom_topic,
                }
            ],
        ),
        optional_node(
            "simbiosys_ui",
            "ui_node",
            "simbiosys_ui is not available; mapping will run without the UI.",
            name="ui_node",
            output="screen",
        ),
        optional_node(
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
            (
                "teleop_twist_keyboard is missing. Install it to drive while "
                "mapping; cmd_vel should target the MIRTE base controller."
            ),
            name="teleop_twist_keyboard",
            output="screen",
            remappings=[("cmd_vel", cmd_vel_topic)],
        ),
    ]

    if package_available("slam_toolbox"):
        actions.append(
            optional_python_launch(
                "slam_toolbox",
                ["launch", "online_async_launch.py"],
                "slam_toolbox is not installed. TODO: install slam_toolbox for mapping.",
                {"slam_params_file": slam_params, "use_sim_time": use_sim_time},
            )
        )
    else:
        actions.append(
            LogInfo(
                msg=(
                    "TODO: slam_toolbox is not found in this environment. Install "
                    "it with rosdep/Pixi before running mapping mode."
                )
            )
        )

    return LaunchDescription(actions)

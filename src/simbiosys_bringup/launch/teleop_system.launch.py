from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from simbiosys_bringup.launch_utils import optional_node


def generate_launch_description():
    """Launch keyboard teleop remapped to the MIRTE base controller."""
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/mirte_base_controller/cmd_vel",
                description="Velocity topic exposed by the MIRTE base controller.",
            ),
            optional_node(
                "teleop_twist_keyboard",
                "teleop_twist_keyboard",
                (
                    "teleop_twist_keyboard is not installed. Install the ROS 2 "
                    "teleop package before using keyboard control."
                ),
                name="teleop_twist_keyboard",
                output="screen",
                remappings=[
                    ("cmd_vel", cmd_vel_topic),
                ],
            ),
        ]
    )

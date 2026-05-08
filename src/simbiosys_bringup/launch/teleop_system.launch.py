from launch import LaunchDescription

from simbiosys_bringup.launch_utils import optional_node


def generate_launch_description():
    """Launch keyboard teleop remapped to the MIRTE base controller."""
    return LaunchDescription(
        [
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
                    ("cmd_vel", "/mirte_base_controller/cmd_vel_unstamped"),
                ],
            ),
        ]
    )

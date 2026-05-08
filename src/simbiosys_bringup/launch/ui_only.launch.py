from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only operator-facing placeholder nodes."""
    return LaunchDescription(
        [
            Node(
                package="simbiosys_ui",
                executable="ui_node",
                name="ui_node",
                output="screen",
            ),
            Node(
                package="simbiosys_ui",
                executable="teleop_interface_node",
                name="teleop_interface_node",
                output="screen",
            ),
        ]
    )

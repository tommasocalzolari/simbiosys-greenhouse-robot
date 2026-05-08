from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch UI with dummy dashboard data for robot-free development."""
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
                executable="dummy_dashboard_data_node",
                name="dummy_dashboard_data_node",
                output="screen",
            ),
        ]
    )

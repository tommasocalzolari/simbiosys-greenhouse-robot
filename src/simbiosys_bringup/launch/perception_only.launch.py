from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the placeholder perception nodes."""
    return LaunchDescription(
        [
            Node(
                package="simbiosys_perception",
                executable="plant_analysis_node",
                name="plant_analysis_node",
                output="screen",
            ),
            Node(
                package="simbiosys_perception",
                executable="flower_detection_node",
                name="flower_detection_node",
                output="screen",
            ),
            Node(
                package="simbiosys_perception",
                executable="obstacle_detection_node",
                name="obstacle_detection_node",
                output="screen",
            ),
            Node(
                package="simbiosys_perception",
                executable="bug_detection_node",
                name="bug_detection_node",
                output="screen",
            ),
        ]
    )

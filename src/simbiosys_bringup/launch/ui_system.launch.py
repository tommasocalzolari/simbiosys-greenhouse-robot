from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch UI with dummy dashboard data for robot-free development."""
    image_topic = LaunchConfiguration("image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="Main RGB camera image topic from the MIRTE Master.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/mirte_base_controller/cmd_vel",
                description="Velocity topic exposed by the MIRTE base controller.",
            ),
            DeclareLaunchArgument(
                "web_host",
                default_value="0.0.0.0",
                description="Host interface for the SimBioSys web UI.",
            ),
            DeclareLaunchArgument(
                "web_port",
                default_value="8080",
                description="TCP port for the SimBioSys web UI.",
            ),
            Node(
                package="simbiosys_ui",
                executable="ui_node",
                name="ui_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "web_host": web_host,
                        "web_port": web_port,
                    }
                ],
            ),
            Node(
                package="simbiosys_ui",
                executable="dummy_dashboard_data_node",
                name="dummy_dashboard_data_node",
                output="screen",
            ),
        ]
    )

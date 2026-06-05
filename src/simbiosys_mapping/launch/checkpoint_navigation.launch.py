from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    annotations_file = LaunchConfiguration("annotations_file")
    command_topic = LaunchConfiguration("command_topic")
    status_topic = LaunchConfiguration("status_topic")
    nav2_action_name = LaunchConfiguration("nav2_action_name")
    nav2_server_timeout_sec = LaunchConfiguration("nav2_server_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "annotations_file",
                default_value="maps/mirte_map_annotations.json",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/checkpoint_commands",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="/checkpoint_status",
            ),
            DeclareLaunchArgument(
                "nav2_action_name",
                default_value="/navigate_to_pose",
            ),
            DeclareLaunchArgument(
                "nav2_server_timeout_sec",
                default_value="5.0",
            ),
            Node(
                package="simbiosys_mapping",
                executable="checkpoint_navigator_node",
                name="checkpoint_navigator_node",
                output="screen",
                parameters=[
                    {
                        "annotations_file": annotations_file,
                        "command_topic": command_topic,
                        "status_topic": status_topic,
                        "nav2_action_name": nav2_action_name,
                        "nav2_server_timeout_sec": ParameterValue(
                            nav2_server_timeout_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    annotations_file = LaunchConfiguration("annotations_file")
    command_topic = LaunchConfiguration("command_topic")
    status_topic = LaunchConfiguration("status_topic")
    nav2_action_name = LaunchConfiguration("nav2_action_name")
    nav2_server_timeout_sec = LaunchConfiguration("nav2_server_timeout_sec")
    marker_topic = LaunchConfiguration("marker_topic")
    publish_markers = LaunchConfiguration("publish_markers")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    initial_pose_publish_count = LaunchConfiguration("initial_pose_publish_count")
    initial_pose_publish_period = LaunchConfiguration("initial_pose_publish_period")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
            DeclareLaunchArgument("marker_topic", default_value="/map_annotations"),
            DeclareLaunchArgument("publish_markers", default_value="true"),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            DeclareLaunchArgument(
                "initial_pose_publish_count",
                default_value="5",
                description=(
                    "Publish a short initial-pose burst without repeatedly "
                    "resetting AMCL during Nav2 startup."
                ),
            ),
            DeclareLaunchArgument("initial_pose_publish_period", default_value="1.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "rviz",
                        "navigation.rviz",
                    ]
                ),
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
                        "marker_topic": marker_topic,
                        "publish_markers": ParameterValue(
                            publish_markers,
                            value_type=bool,
                        ),
                        "publish_initial_pose": ParameterValue(
                            publish_initial_pose,
                            value_type=bool,
                        ),
                        "initial_pose_publish_count": ParameterValue(
                            initial_pose_publish_count,
                            value_type=int,
                        ),
                        "initial_pose_publish_period": ParameterValue(
                            initial_pose_publish_period,
                            value_type=float,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_checkpoint_navigation",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

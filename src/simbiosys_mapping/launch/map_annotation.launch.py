from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_yaml = LaunchConfiguration("map_yaml")
    annotations_file = LaunchConfiguration("annotations_file")
    map_topic = LaunchConfiguration("map_topic")
    start_annotation_on_startup = LaunchConfiguration("start_annotation_on_startup")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_yaml", default_value="maps/mirte_map.yaml"),
            DeclareLaunchArgument(
                "annotations_file",
                default_value="maps/mirte_map_annotations.json",
            ),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "start_annotation_on_startup",
                default_value="true",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "rviz",
                        "map_annotation.rviz",
                    ]
                ),
            ),
            Node(
                package="simbiosys_mapping",
                executable="map_annotation_node",
                name="map_annotation_node",
                output="screen",
                parameters=[
                    {
                        "map_yaml": map_yaml,
                        "annotations_file": annotations_file,
                        "map_topic": map_topic,
                        "start_annotation_on_startup": ParameterValue(
                            start_annotation_on_startup,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )

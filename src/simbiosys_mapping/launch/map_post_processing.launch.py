from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_yaml = LaunchConfiguration("map_yaml")
    map_topic = LaunchConfiguration("map_topic")
    annotation_name = LaunchConfiguration("annotation_name")
    process_on_startup = LaunchConfiguration("process_on_startup")
    start_annotation_after_processing = LaunchConfiguration(
        "start_annotation_after_processing"
    )
    min_occupied_cluster_size = LaunchConfiguration("min_occupied_cluster_size")
    straighten_kernel_size = LaunchConfiguration("straighten_kernel_size")
    closed_obstacle_min_area = LaunchConfiguration("closed_obstacle_min_area")
    closed_obstacle_max_area_ratio = LaunchConfiguration(
        "closed_obstacle_max_area_ratio"
    )
    rectangularize_closed_obstacles = LaunchConfiguration(
        "rectangularize_closed_obstacles"
    )
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_yaml", default_value="maps/mirte_map.yaml"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "annotation_name",
                default_value="mirte_map_annotations",
            ),
            DeclareLaunchArgument("process_on_startup", default_value="true"),
            DeclareLaunchArgument(
                "start_annotation_after_processing",
                default_value="true",
            ),
            DeclareLaunchArgument("min_occupied_cluster_size", default_value="2"),
            DeclareLaunchArgument("straighten_kernel_size", default_value="5"),
            DeclareLaunchArgument("closed_obstacle_min_area", default_value="8"),
            DeclareLaunchArgument(
                "closed_obstacle_max_area_ratio",
                default_value="0.15",
            ),
            DeclareLaunchArgument(
                "rectangularize_closed_obstacles",
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
                executable="map_post_processor_node",
                name="map_post_processor_node",
                output="screen",
                parameters=[
                    {
                        "map_yaml": map_yaml,
                        "map_topic": map_topic,
                        "annotation_name": annotation_name,
                        "process_on_startup": ParameterValue(
                            process_on_startup,
                            value_type=bool,
                        ),
                        "start_annotation_after_processing": ParameterValue(
                            start_annotation_after_processing,
                            value_type=bool,
                        ),
                        "min_occupied_cluster_size": ParameterValue(
                            min_occupied_cluster_size,
                            value_type=int,
                        ),
                        "straighten_kernel_size": ParameterValue(
                            straighten_kernel_size,
                            value_type=int,
                        ),
                        "closed_obstacle_min_area": ParameterValue(
                            closed_obstacle_min_area,
                            value_type=int,
                        ),
                        "closed_obstacle_max_area_ratio": ParameterValue(
                            closed_obstacle_max_area_ratio,
                            value_type=float,
                        ),
                        "rectangularize_closed_obstacles": ParameterValue(
                            rectangularize_closed_obstacles,
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

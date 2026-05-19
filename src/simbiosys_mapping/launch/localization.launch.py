from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = LaunchConfiguration("simulation")
    use_sim_time = PythonExpression(
        [
            "'",
            LaunchConfiguration("use_sim_time"),
            "' if '",
            LaunchConfiguration("use_sim_time"),
            "' != 'auto' else '",
            simulation,
            "'",
        ]
    )
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    world = LaunchConfiguration("world")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    rviz_config = LaunchConfiguration("rviz_config")
    scan_topic = LaunchConfiguration("scan_topic")

    def launch_gazebo_if_simulation(context):
        if simulation.perform(context).lower() not in ("true", "1", "yes"):
            return []
        return [
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("mirte_gazebo"),
                                "launch",
                                "gazebo_mirte_master_empty.launch.xml",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "world": world,
                    "gui": gazebo_gui,
                }.items(),
            )
        ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            },
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "scan_topic": scan_topic,
            },
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "autostart": LaunchConfiguration("autostart"),
                "node_names": ["map_server", "amcl"],
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_pose_node = Node(
        package="simbiosys_mapping",
        executable="initial_pose_node",
        name="initial_pose_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "enabled": publish_initial_pose,
                "x": LaunchConfiguration("initial_pose_x"),
                "y": LaunchConfiguration("initial_pose_y"),
                "yaw": LaunchConfiguration("initial_pose_yaw"),
                "publish_period": LaunchConfiguration("initial_pose_period"),
                "publish_count": LaunchConfiguration("initial_pose_count"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description=(
                    "true: start Gazebo with the static obstacle world. "
                    "false: use real robot topics and do not start Gazebo."
                ),
            ),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "worlds",
                        "static_obstacles.world",
                    ]
                ),
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "rviz",
                        "localization.rviz",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                description=(
                    "auto follows simulation. Set true/false to override."
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value="maps/mirte_map.yaml",
                description="Saved map YAML file, usually outside this package.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "config",
                        "amcl_localization.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "publish_initial_pose",
                default_value="false",
                description=(
                    "false by default so the operator sets AMCL's initial "
                    "pose manually in RViz. Set true only for scripted tests."
                ),
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_period", default_value="1.0"),
            DeclareLaunchArgument("initial_pose_count", default_value="10"),
            OpaqueFunction(function=launch_gazebo_if_simulation),
            map_server,
            amcl,
            lifecycle_manager,
            initial_pose_node,
            rviz_node,
        ]
    )

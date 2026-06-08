from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


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
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    localization_params_file = PathJoinSubstitution(
        [
            FindPackageShare("simbiosys_mapping"),
            "config",
            "amcl_localization.yaml",
        ]
    )
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    cmd_vel_topic = PythonExpression(
        [
            "'",
            LaunchConfiguration("cmd_vel_topic"),
            "' if '",
            LaunchConfiguration("cmd_vel_topic"),
            "' != 'auto' else "
            "('/mirte_base_controller/cmd_vel_unstamped' if '",
            simulation,
            "'.lower() in ('true', '1', 'yes') "
            "else '/mirte_base_controller/cmd_vel')",
        ]
    )
    odom_topic = LaunchConfiguration("odom_topic")
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    scan_topic = LaunchConfiguration("scan_topic")
    nav_to_pose_bt_xml = PathJoinSubstitution(
        [
            FindPackageShare("simbiosys_mapping"),
            "behavior_trees",
            "nav2_tight_space_backup_bt_deadband.xml",
        ]
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "odom_topic": odom_topic,
                "default_nav_to_pose_bt_xml": nav_to_pose_bt_xml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

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
            localization_params_file,
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
            localization_params_file,
            {
                "use_sim_time": use_sim_time,
                "scan_topic": scan_topic,
            },
        ],
    )

    localization_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            localization_params_file,
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server", "amcl"],
            },
        ],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings + [("cmd_vel", cmd_vel_topic)],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description=(
                    "true: start Gazebo, localization, and navigation. "
                    "false: use real robot topics and start localization/navigation."
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
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                description="auto follows simulation. Set true/false to override.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "config",
                        "nav2_navigation.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value="maps/mirte_map.yaml",
                description="Saved map YAML file used by Nav2 map_server.",
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="auto",
                description=(
                    "auto uses /mirte_base_controller/cmd_vel_unstamped in "
                    "simulation and /mirte_base_controller/cmd_vel on the real "
                    "robot. Pass a topic name to override."
                ),
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/mirte_base_controller/odom",
                description="Odometry topic used by Nav2 controller/BT logic.",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
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
            OpaqueFunction(function=launch_gazebo_if_simulation),
            map_server,
            amcl,
            localization_lifecycle_manager,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
            rviz_node,
        ]
    )

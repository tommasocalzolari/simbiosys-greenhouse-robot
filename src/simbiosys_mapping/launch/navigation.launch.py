from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "odom_topic": odom_topic,
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
                    "Only controls time source here. Start simulation/localization "
                    "separately before launching navigation."
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
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
            rviz_node,
        ]
    )

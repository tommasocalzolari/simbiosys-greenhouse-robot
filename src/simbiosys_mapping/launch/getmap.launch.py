from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = LaunchConfiguration("simulation")
    slam_params_file = LaunchConfiguration("slam_params_file")
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
    world = LaunchConfiguration("world")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    rviz_config = LaunchConfiguration("rviz_config")

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("slam_toolbox"),
                        "launch",
                        "online_async_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

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

    getmap_node = Node(
        package="simbiosys_mapping",
        executable="getmap_node",
        name="getmap_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "scan_topic": LaunchConfiguration("scan_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "map_topic": LaunchConfiguration("map_topic"),
                "output_dir": LaunchConfiguration("output_dir"),
                "map_name": LaunchConfiguration("map_name"),
                "auto_save_period": LaunchConfiguration("auto_save_period"),
                "save_on_shutdown": LaunchConfiguration("save_on_shutdown"),
            }
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="true",
                description=(
                    "true: start Gazebo with the static obstacle world. "
                    "false: do not start Gazebo; use real robot topics."
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
                        "getmap.rviz",
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
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "config",
                        "slam_toolbox_mapping.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("output_dir", default_value="maps"),
            DeclareLaunchArgument("map_name", default_value="mirte_map"),
            DeclareLaunchArgument("auto_save_period", default_value="20.0"),
            DeclareLaunchArgument("save_on_shutdown", default_value="true"),
            OpaqueFunction(function=launch_gazebo_if_simulation),
            slam_toolbox_launch,
            getmap_node,
            rviz_node,
        ]
    )

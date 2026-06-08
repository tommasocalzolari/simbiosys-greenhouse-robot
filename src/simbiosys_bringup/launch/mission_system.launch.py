from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Nav2 plus the SimBioSys mission/perception stack."""
    simulation = LaunchConfiguration("simulation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    start_rviz = LaunchConfiguration("start_rviz")
    image_topic = LaunchConfiguration("image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    bed_side_enable_motion = LaunchConfiguration("bed_side_enable_motion")
    operator_led_enabled = LaunchConfiguration("operator_led_enabled")
    start_ui = LaunchConfiguration("start_ui")
    start_bed_side_alignment = LaunchConfiguration("start_bed_side_alignment")
    bed_side_surface_region = LaunchConfiguration("bed_side_surface_region")

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "launch",
                        "navigation.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "simulation": simulation,
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "start_rviz": start_rviz,
            "cmd_vel_topic": cmd_vel_topic,
            "odom_topic": odom_topic,
            "scan_topic": scan_topic,
        }.items(),
    )

    laptop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_bringup"),
                        "launch",
                        "laptop_system.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "image_topic": image_topic,
            "cmd_vel_topic": cmd_vel_topic,
            "odom_topic": odom_topic,
            "scan_topic": scan_topic,
            "bed_side_enable_motion": bed_side_enable_motion,
            "operator_led_enabled": operator_led_enabled,
            "start_ui": start_ui,
            "start_bed_side_alignment": start_bed_side_alignment,
            "bed_side_surface_region": bed_side_surface_region,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="true starts Gazebo through the Nav2 launch; false uses robot topics.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                description="auto follows simulation in the navigation launch.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="maps/mirte_map_REAL.yaml",
                description="Saved map YAML used by Nav2 map_server.",
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="RGB camera image topic; flower detection subscribes to /compressed by default.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/mirte_base_controller/cmd_vel",
                description="Velocity command topic for Nav2 and SimBioSys alignment controllers.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/mirte_base_controller/odom",
                description="Odometry topic consumed by Nav2 and mission health checks.",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "bed_side_enable_motion",
                default_value="false",
                description="Keep false while testing; true enables physical alignment cmd_vel output.",
            ),
            DeclareLaunchArgument(
                "operator_led_enabled",
                default_value="false",
                description="Start operator LED feedback; requires mirte_msgs.",
            ),
            DeclareLaunchArgument(
                "start_ui",
                default_value="true",
                description="Start the SimBioSys web UI from this launch file.",
            ),
            DeclareLaunchArgument(
                "start_bed_side_alignment",
                default_value="true",
                description="Start LaserScan-based bed-side alignment perception.",
            ),
            DeclareLaunchArgument(
                "bed_side_surface_region",
                default_value="left",
                description="LaserScan region used for bed-side alignment: left, right, or front.",
            ),
            navigation_launch,
            laptop_launch,
        ]
    )

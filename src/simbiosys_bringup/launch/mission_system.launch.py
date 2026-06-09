from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    operator_led_enabled = LaunchConfiguration("operator_led_enabled")
    start_ui = LaunchConfiguration("start_ui")
    start_bed_side_alignment = LaunchConfiguration("start_bed_side_alignment")
    bed_side_surface_region = LaunchConfiguration("bed_side_surface_region")
    annotations_file = LaunchConfiguration("annotations_file")
    checkpoint_status_topic = LaunchConfiguration("checkpoint_status_topic")
    checkpoint_command_topic = LaunchConfiguration("checkpoint_command_topic")
    checkpoint_ready_timeout_sec = LaunchConfiguration("checkpoint_ready_timeout_sec")
    alignment_enable_motion = LaunchConfiguration("alignment_enable_motion")
    plant_analysis_dry_run = LaunchConfiguration("plant_analysis_dry_run")
    flower_model_path = LaunchConfiguration("flower_model_path")

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

    checkpoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("simbiosys_mapping"),
                        "launch",
                        "checkpoint_navigation.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "annotations_file": annotations_file,
            "command_topic": checkpoint_command_topic,
            "status_topic": checkpoint_status_topic,
            "nav2_action_name": "navigate_to_pose",
            "use_sim_time": use_sim_time,
            "use_rviz": "false",
        }.items(),
    )

    wait_for_checkpoint_ready = Node(
        package="simbiosys_bringup",
        executable="wait_for_checkpoint_ready",
        name="wait_for_checkpoint_ready",
        output="screen",
        parameters=[
            {
                "status_topic": checkpoint_status_topic,
                "timeout_sec": ParameterValue(
                    checkpoint_ready_timeout_sec,
                    value_type=float,
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
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
            "bed_side_enable_motion": alignment_enable_motion,
            "operator_led_enabled": operator_led_enabled,
            "start_ui": start_ui,
            "start_bed_side_alignment": start_bed_side_alignment,
            "bed_side_surface_region": bed_side_surface_region,
            "plant_analysis_dry_run": plant_analysis_dry_run,
            "flower_model_path": flower_model_path,
            "use_sim_time": use_sim_time,
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
                default_value="false",
                description="Use simulation clock for mission, checkpoint, and navigation nodes.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="maps/mirte_map.yaml",
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
                description="Deprecated alias; use alignment_enable_motion.",
            ),
            DeclareLaunchArgument(
                "alignment_enable_motion",
                default_value="false",
                description="Keep false while testing; true enables physical alignment cmd_vel output.",
            ),
            DeclareLaunchArgument(
                "plant_analysis_dry_run",
                default_value="true",
                description="Keep true until the camera and YOLO model are validated.",
            ),
            DeclareLaunchArgument(
                "flower_model_path",
                default_value="models/flower_model.pt",
                description="YOLO model path used by flower_detection_node.",
            ),
            DeclareLaunchArgument(
                "annotations_file",
                default_value="maps/mirte_map_annotations.json",
                description="Checkpoint annotations with bed/side scan metadata.",
            ),
            DeclareLaunchArgument(
                "checkpoint_command_topic",
                default_value="/checkpoint_commands",
            ),
            DeclareLaunchArgument(
                "checkpoint_status_topic",
                default_value="/checkpoint_status",
            ),
            DeclareLaunchArgument(
                "checkpoint_ready_timeout_sec",
                default_value="0.0",
                description="0 waits indefinitely for checkpoint readiness before starting mission nodes.",
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
                default_value="front",
                description="LaserScan region used for bed-side alignment: left, right, or front.",
            ),
            navigation_launch,
            checkpoint_launch,
            wait_for_checkpoint_ready,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_checkpoint_ready,
                    on_exit=[laptop_launch],
                )
            ),
        ]
    )

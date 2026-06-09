from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    simulation = LaunchConfiguration("simulation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    annotations_file = LaunchConfiguration("annotations_file")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    analysis_dry_run = LaunchConfiguration("analysis_dry_run")
    start_ui = LaunchConfiguration("start_ui")
    start_rviz = LaunchConfiguration("start_rviz")
    operator_led_enabled = LaunchConfiguration("operator_led_enabled")
    operator_led_single_service_name = LaunchConfiguration(
        "operator_led_single_service_name"
    )
    operator_led_brightness = LaunchConfiguration("operator_led_brightness")
    operator_led_turn_angular_threshold = LaunchConfiguration(
        "operator_led_turn_angular_threshold"
    )
    operator_led_strafe_linear_y_threshold = LaunchConfiguration(
        "operator_led_strafe_linear_y_threshold"
    )
    operator_led_manual_control_topic = LaunchConfiguration(
        "operator_led_manual_control_topic"
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("simbiosys_mapping"),
                    "launch",
                    "navigation.launch.py",
                ]
            )
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

    checkpoint_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("simbiosys_mapping"),
                    "launch",
                    "checkpoint_navigation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "annotations_file": annotations_file,
            "use_sim_time": use_sim_time,
            "use_rviz": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value="maps/mirte_map.yaml"),
            DeclareLaunchArgument(
                "annotations_file",
                default_value="maps/mirte_map_annotations.json",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/depth/image_raw",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/mirte_base_controller/odom",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/mirte_base_controller/cmd_vel",
            ),
            DeclareLaunchArgument("analysis_dry_run", default_value="false"),
            DeclareLaunchArgument("start_ui", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument(
                "operator_led_enabled",
                default_value="true",
                description="Start MIRTE Neopixel operator status feedback.",
            ),
            DeclareLaunchArgument(
                "operator_led_single_service_name",
                default_value="/io/leds/leds/set_color_single",
                description="MIRTE Neopixel single-LED service.",
            ),
            DeclareLaunchArgument(
                "operator_led_brightness",
                default_value="0.35",
                description="Operator LED brightness in the range 0.0-1.0.",
            ),
            DeclareLaunchArgument(
                "operator_led_turn_angular_threshold",
                default_value="0.15",
                description="Minimum absolute angular velocity for turn blinkers.",
            ),
            DeclareLaunchArgument(
                "operator_led_strafe_linear_y_threshold",
                default_value="0.03",
                description="Minimum absolute lateral velocity for strafe blinkers.",
            ),
            DeclareLaunchArgument(
                "operator_led_manual_control_topic",
                default_value="simbiosys/ui/manual_control_active",
                description="Latched topic indicating whether UI teleop owns control.",
            ),
            navigation,
            checkpoint_navigation,
            Node(
                package="simbiosys_perception",
                executable="flower_detection_node",
                name="flower_detection_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "depth_topic": depth_topic,
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="simbiosys_behavior",
                executable="demo_checkpoint_mission_node",
                name="demo_checkpoint_mission_node",
                output="screen",
                parameters=[
                    {
                        "analysis_dry_run": ParameterValue(
                            analysis_dry_run,
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="simbiosys_behavior",
                executable="operator_led_node",
                name="operator_led_node",
                output="screen",
                condition=IfCondition(operator_led_enabled),
                parameters=[
                    {
                        "enabled": ParameterValue(
                            operator_led_enabled,
                            value_type=bool,
                        ),
                        "set_single_service_name": operator_led_single_service_name,
                        "cmd_vel_topic": cmd_vel_topic,
                        "manual_control_topic": operator_led_manual_control_topic,
                        "brightness": ParameterValue(
                            operator_led_brightness,
                            value_type=float,
                        ),
                        "turn_angular_threshold": ParameterValue(
                            operator_led_turn_angular_threshold,
                            value_type=float,
                        ),
                        "strafe_linear_y_threshold": ParameterValue(
                            operator_led_strafe_linear_y_threshold,
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
                package="simbiosys_ui",
                executable="ui_node",
                name="ui_node",
                output="screen",
                condition=IfCondition(start_ui),
                parameters=[
                    {
                        "image_topic": image_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )

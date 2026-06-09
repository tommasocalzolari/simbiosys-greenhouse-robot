from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the laptop-side SimBioSys stack.

    The MIRTE hardware bringup is expected to be started separately on the
    robot, usually over SSH, before this launch file is used on the laptop.
    """
    image_topic = LaunchConfiguration("image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_close_position = LaunchConfiguration("gripper_close_position")
    gripper_min_position = LaunchConfiguration("gripper_min_position")
    gripper_max_position = LaunchConfiguration("gripper_max_position")
    gripper_max_effort = LaunchConfiguration("gripper_max_effort")
    motion_duration_sec = LaunchConfiguration("motion_duration_sec")
    bed_side_enable_motion = LaunchConfiguration("bed_side_enable_motion")
    operator_led_enabled = LaunchConfiguration("operator_led_enabled")
    start_ui = LaunchConfiguration("start_ui")
    start_bed_side_alignment = LaunchConfiguration("start_bed_side_alignment")
    bed_side_surface_region = LaunchConfiguration("bed_side_surface_region")
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_duration_sec",
                default_value="3.0",
                description="Seconds used by the named arm pose wrapper to reach the target.",
            ),
            DeclareLaunchArgument(
                "bed_side_enable_motion",
                default_value="false",
                description=(
                    "Enable physical cmd_vel output from the bed-side controller. "
                    "Keep false until perception alignment and robot safety are validated."
                ),
            ),
            DeclareLaunchArgument(
                "operator_led_enabled",
                default_value="false",
                description=(
                    "Start MIRTE Neopixel operator status feedback. Requires "
                    "mirte_msgs to be installed in this environment."
                ),
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
            DeclareLaunchArgument(
                "operator_led_single_service_name",
                default_value="/io/leds/leds/set_color_single",
                description="MIRTE Neopixel single-LED service used for operator feedback.",
            ),
            DeclareLaunchArgument(
                "operator_led_brightness",
                default_value="0.35",
                description="Brightness scale for operator LED colors in the range 0.0-1.0.",
            ),
            DeclareLaunchArgument(
                "operator_led_turn_angular_threshold",
                default_value="0.15",
                description="Minimum absolute cmd_vel angular.z for turn blinkers.",
            ),
            DeclareLaunchArgument(
                "operator_led_strafe_linear_y_threshold",
                default_value="0.03",
                description="Minimum absolute cmd_vel linear.y for strafe blinkers.",
            ),
            DeclareLaunchArgument(
                "operator_led_manual_control_topic",
                default_value="simbiosys/ui/manual_control_active",
                description=(
                    "Latched Bool topic indicating whether UI teleop owns control."
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="Main RGB camera image topic from the MIRTE Master.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/mirte_base_controller/cmd_vel",
                description="Velocity topic exposed by the MIRTE base controller.",
            ),
            DeclareLaunchArgument(
                "web_host",
                default_value="0.0.0.0",
                description="Host interface for the SimBioSys web UI.",
            ),
            DeclareLaunchArgument(
                "web_port",
                default_value="8080",
                description="TCP port for the SimBioSys web UI.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/mirte_base_controller/odom",
                description="Odometry topic published by the MIRTE base controller.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="Laser scan topic from the MIRTE Master.",
            ),
            DeclareLaunchArgument(
                "gripper_open_position",
                default_value="0.04",
                description="Placeholder open position for gripper_joint.",
            ),
            DeclareLaunchArgument(
                "gripper_close_position",
                default_value="0.0",
                description="Placeholder close position for gripper_joint.",
            ),
            DeclareLaunchArgument(
                "gripper_min_position",
                default_value="-0.7603",
                description="Observed lower gripper_joint limit on the MIRTE Master.",
            ),
            DeclareLaunchArgument(
                "gripper_max_position",
                default_value="0.6458",
                description="Observed upper gripper_joint limit on the MIRTE Master.",
            ),
            DeclareLaunchArgument(
                "gripper_max_effort",
                default_value="0.0",
                description="Max effort sent to the GripperCommand action.",
            ),
            Node(
                package="simbiosys_behavior",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
            ),
            Node(
                package="simbiosys_behavior",
                executable="bed_side_controller_node",
                name="bed_side_controller_node",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "enable_motion": bed_side_enable_motion,
                    }
                ],
            ),
            Node(
                package="simbiosys_behavior",
                executable="scan_pose_controller_node",
                name="scan_pose_controller_node",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "enable_motion": bed_side_enable_motion,
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
                        "enabled": operator_led_enabled,
                        "set_single_service_name": operator_led_single_service_name,
                        "cmd_vel_topic": cmd_vel_topic,
                        "manual_control_topic": operator_led_manual_control_topic,
                        "brightness": operator_led_brightness,
                        "turn_angular_threshold": operator_led_turn_angular_threshold,
                        "strafe_linear_y_threshold": operator_led_strafe_linear_y_threshold,
                    }
                ],
            ),
            Node(
                package="simbiosys_perception",
                executable="bed_side_alignment_node",
                name="bed_side_alignment_node",
                output="screen",
                condition=IfCondition(start_bed_side_alignment),
                parameters=[
                    {
                        "scan_topic": scan_topic,
                        "surface_region": bed_side_surface_region,
                    }
                ],
            ),
            Node(
                package="simbiosys_perception",
                executable="flower_detection_node",
                name="flower_detection_node",
                output="screen",
                parameters=[{"image_topic": image_topic}],
            ),
            Node(
                package="simbiosys_mapping",
                executable="mapping_status_node",
                name="mapping_status_node",
                output="screen",
                parameters=[
                    {
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                    }
                ],
            ),
            Node(
                package="simbiosys_base",
                executable="path_generation_node",
                name="path_generation_node",
                output="screen",
            ),
            Node(
                package="simbiosys_base",
                executable="path_execution_node",
                name="path_execution_node",
                output="screen",
            ),
            Node(
                package="simbiosys_arm",
                executable="named_joint_pose_node",
                name="named_joint_pose_node",
                output="screen",
                parameters=[{"motion_duration_sec": motion_duration_sec}],
            ),
            Node(
                package="simbiosys_arm",
                executable="gripper_client_node",
                name="gripper_client_node",
                output="screen",
                parameters=[
                    {
                        "open_position": gripper_open_position,
                        "close_position": gripper_close_position,
                        "min_position": gripper_min_position,
                        "max_position": gripper_max_position,
                        "max_effort": gripper_max_effort,
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
                        "web_host": web_host,
                        "web_port": web_port,
                    }
                ],
            ),
        ]
    )

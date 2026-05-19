from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the laptop-side SimBioSys stack.

    The MIRTE hardware bringup is expected to be started separately on the
    robot, usually over SSH, before this launch file is used on the laptop.
    """
    image_topic = LaunchConfiguration("image_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_close_position = LaunchConfiguration("gripper_close_position")
    gripper_min_position = LaunchConfiguration("gripper_min_position")
    gripper_max_position = LaunchConfiguration("gripper_max_position")
    gripper_max_effort = LaunchConfiguration("gripper_max_effort")
    motion_duration_sec = LaunchConfiguration("motion_duration_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_duration_sec",
                default_value="3.0",
                description="Seconds used by the named arm pose wrapper to reach the target.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="Main RGB camera image topic from the MIRTE Master.",
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
                executable="joint_state_monitor_node",
                name="joint_state_monitor_node",
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
            ),
        ]
    )

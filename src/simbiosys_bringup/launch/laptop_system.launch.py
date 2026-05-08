from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the laptop-side SimBioSys stack.

    The MIRTE hardware bringup is expected to be started separately on the
    robot, usually over SSH, before this launch file is used on the laptop.
    """
    return LaunchDescription(
        [
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
            ),
            Node(
                package="simbiosys_mapping",
                executable="mapping_status_node",
                name="mapping_status_node",
                output="screen",
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
            ),
            Node(
                package="simbiosys_arm",
                executable="gripper_client_node",
                name="gripper_client_node",
                output="screen",
            ),
            Node(
                package="simbiosys_ui",
                executable="ui_node",
                name="ui_node",
                output="screen",
            ),
        ]
    )

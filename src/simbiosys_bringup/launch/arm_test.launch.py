from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch safe arm wrapper nodes for interface testing."""
    return LaunchDescription(
        [
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
        ]
    )

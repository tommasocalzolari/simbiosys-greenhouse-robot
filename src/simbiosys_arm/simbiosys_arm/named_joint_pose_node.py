import rclpy
from rclpy.node import Node
from simbiosys_interfaces.srv import SendNamedArmPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_joint",
]

SAFE_PLACEHOLDER_POSES = {
    "home": [0.0, 0.0, 0.0, 0.0],
    "inspect": [0.0, -0.35, 0.55, -0.20],
    "stow": [0.0, 0.25, -0.25, 0.0],
}


class NamedJointPoseNode(Node):
    """Small wrapper that sends named placeholder poses to the MIRTE arm topic."""

    def __init__(self) -> None:
        super().__init__("named_joint_pose_node")
        self.declare_parameter(
            "arm_trajectory_topic",
            "/mirte_master_arm_controller/joint_trajectory",
        )
        self.declare_parameter("motion_duration_sec", 3.0)

        self._trajectory_topic = (
            self.get_parameter("arm_trajectory_topic").get_parameter_value().string_value
        )
        self._motion_duration_sec = (
            self.get_parameter("motion_duration_sec").get_parameter_value().double_value
        )

        self._publisher = self.create_publisher(
            JointTrajectory,
            self._trajectory_topic,
            10,
        )
        self.create_service(
            SendNamedArmPose,
            "simbiosys/send_named_arm_pose",
            self._on_send_named_pose,
        )

        self.get_logger().info(
            "Named arm pose wrapper ready. Available safe placeholders: "
            + ", ".join(sorted(SAFE_PLACEHOLDER_POSES))
        )

    def _on_send_named_pose(self, request, response):
        pose_name = request.pose_name.strip().lower()
        if pose_name not in SAFE_PLACEHOLDER_POSES:
            response.accepted = False
            response.message = (
                f"Unknown pose '{request.pose_name}'. Known poses: "
                + ", ".join(sorted(SAFE_PLACEHOLDER_POSES))
            )
            return response

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = SAFE_PLACEHOLDER_POSES[pose_name]
        point.time_from_start.sec = int(self._motion_duration_sec)
        point.time_from_start.nanosec = int(
            (self._motion_duration_sec - int(self._motion_duration_sec)) * 1e9
        )
        trajectory.points = [point]

        self._publisher.publish(trajectory)
        response.accepted = True
        response.message = (
            f"Published safe placeholder pose '{pose_name}' to {self._trajectory_topic}"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NamedJointPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

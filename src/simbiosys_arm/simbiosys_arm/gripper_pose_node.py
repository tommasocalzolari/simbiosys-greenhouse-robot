import rclpy
from rclpy.node import Node


class GripperPoseNode(Node):
    """Placeholder gripper pose estimation node."""

    def __init__(self) -> None:
        super().__init__("gripper_pose_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Convert plant analysis into a target gripper pose.
        # TODO: Publish or serve candidate grasp poses for harvesting.
        self.get_logger().info("Gripper pose placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for gripper pose implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

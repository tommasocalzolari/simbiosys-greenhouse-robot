import rclpy
from rclpy.node import Node


class ArmMotionNode(Node):
    """Placeholder high-level arm motion node."""

    def __init__(self) -> None:
        super().__init__("arm_motion_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Add ExecuteArmMotion action server.
        # TODO: Bridge high-level target poses to the robot-side arm controller.
        # TODO: Keep low-level servo control on the MIRTE robot.
        self.get_logger().info("Arm motion placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for arm motion implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmMotionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

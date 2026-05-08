import rclpy
from rclpy.node import Node


class WristCameraPositionNode(Node):
    """Placeholder wrist camera positioning node."""

    def __init__(self) -> None:
        super().__init__("wrist_camera_position_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Choose camera viewpoints for plant inspection.
        # TODO: Request high-level arm motion through ExecuteArmMotion.
        self.get_logger().info("Wrist camera positioning placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for wrist camera positioning implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WristCameraPositionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node


class PathGenerationNode(Node):
    """Placeholder high-level path generation node."""

    def __init__(self) -> None:
        super().__init__("path_generation_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Consume obstacle/environment summaries.
        # TODO: Produce target poses or paths for the robot base.
        self.get_logger().info("Path generation placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for path generation implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathGenerationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node


class PathExecutionNode(Node):
    """Placeholder high-level path execution node."""

    def __init__(self) -> None:
        super().__init__("path_execution_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Add action server/client wiring for MoveToTarget.
        # TODO: Send high-level commands only; low-level motor control stays on MIRTE.
        self.get_logger().info("Path execution placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for path execution implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathExecutionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

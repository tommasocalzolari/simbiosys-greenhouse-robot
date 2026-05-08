import rclpy
from rclpy.node import Node


class BugDetectionNode(Node):
    """Placeholder bug detection node."""

    def __init__(self) -> None:
        super().__init__("bug_detection_node")
        self._timer = self.create_timer(3.0, self._on_timer)

        # TODO: Subscribe to plant/camera observations.
        # TODO: Publish bug detection results or merge them into PlantAnalysis.
        self.get_logger().info("Bug detection placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for bug detection implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BugDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node


class ObstacleDetectionNode(Node):
    """Placeholder obstacle detection node."""

    def __init__(self) -> None:
        super().__init__("obstacle_detection_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Subscribe to LiDAR/depth topics from the MIRTE robot.
        # TODO: Publish obstacle summaries for path generation.
        self.get_logger().info("Obstacle detection placeholder started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for obstacle detection implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

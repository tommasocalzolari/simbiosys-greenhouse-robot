import rclpy
from rclpy.node import Node

from simbiosys_interfaces.msg import FlowerData, TaskStatus


class DummyDashboardDataNode(Node):
    """Publish dashboard data so UI work can continue without robot or Gazebo."""

    def __init__(self) -> None:
        super().__init__("dummy_dashboard_data_node")
        self._status_publisher = self.create_publisher(
            TaskStatus,
            "simbiosys/task_status",
            10,
        )
        self._flower_publisher = self.create_publisher(
            FlowerData,
            "simbiosys/flower_data",
            10,
        )
        self.create_timer(2.0, self._on_timer)
        self.get_logger().info("Publishing dummy dashboard data")

    def _on_timer(self) -> None:
        status = TaskStatus()
        status.current_state = "WAIT_FOR_OPERATOR"
        status.active = True
        status.error = False
        status.message = "Dummy development data"
        self._status_publisher.publish(status)

        flower = FlowerData()
        flower.detected = False
        flower.confidence = 0.0
        flower.label = "placeholder"
        flower.message = "No real camera processing yet"
        self._flower_publisher.publish(flower)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyDashboardDataNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

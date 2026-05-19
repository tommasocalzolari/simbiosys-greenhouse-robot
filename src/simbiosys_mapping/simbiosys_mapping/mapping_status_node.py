import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from simbiosys_interfaces.msg import MappingStatus


class MappingStatusNode(Node):
    """Report whether the expected mapping topics are alive."""

    def __init__(self) -> None:
        super().__init__("mapping_status_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/mirte_base_controller/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("active_map", "")

        self._seen = {"scan": False, "odom": False, "map": False}
        self._status_publisher = self.create_publisher(
            MappingStatus,
            "simbiosys/mapping_status",
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").get_parameter_value().string_value,
            lambda _msg: self._mark_seen("scan"),
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            lambda _msg: self._mark_seen("odom"),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").get_parameter_value().string_value,
            lambda _msg: self._mark_seen("map"),
            10,
        )
        self.create_timer(5.0, self._on_timer)
        self.get_logger().info("Mapping status helper started")

    def _mark_seen(self, key: str) -> None:
        self._seen[key] = True

    def _on_timer(self) -> None:
        status = ", ".join(f"{key}={value}" for key, value in self._seen.items())
        status_msg = MappingStatus()
        status_msg.scan_seen = self._seen["scan"]
        status_msg.odom_seen = self._seen["odom"]
        status_msg.map_seen = self._seen["map"]
        status_msg.localized = status_msg.odom_seen and status_msg.map_seen
        status_msg.active_map = (
            self.get_parameter("active_map").get_parameter_value().string_value
        )
        status_msg.message = f"Mapping topic status: {status}"
        self._status_publisher.publish(status_msg)
        self.get_logger().info(f"Mapping topic status: {status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

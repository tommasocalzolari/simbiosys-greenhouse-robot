import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


class InitialPoseNode(Node):
    """Publish an initial AMCL pose estimate a few times at startup."""

    def __init__(self) -> None:
        super().__init__("initial_pose_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("covariance_xy", 0.25)
        self.declare_parameter("covariance_yaw", 0.0685)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("publish_count", 10)

        self._enabled = self._bool_parameter("enabled")
        self._remaining = self._int_parameter("publish_count")
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

        if not self._enabled:
            self.get_logger().info("Initial pose publisher disabled")
            return

        self.create_timer(self._double_parameter("publish_period"), self._publish)
        self.get_logger().info("Initial pose publisher enabled")

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("true", "1", "yes")
        return bool(value)

    def _double_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _publish(self) -> None:
        if self._remaining <= 0:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.pose.position.x = self._double_parameter("x")
        msg.pose.pose.position.y = self._double_parameter("y")
        msg.pose.pose.orientation = self._quaternion_from_yaw(
            self._double_parameter("yaw")
        )

        covariance_xy = self._double_parameter("covariance_xy")
        covariance_yaw = self._double_parameter("covariance_yaw")
        msg.pose.covariance[0] = covariance_xy
        msg.pose.covariance[7] = covariance_xy
        msg.pose.covariance[35] = covariance_yaw

        self._publisher.publish(msg)
        self._remaining -= 1
        self.get_logger().info(
            "Published initial pose "
            f"x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}"
        )

    def _quaternion_from_yaw(self, yaw: float) -> Quaternion:
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

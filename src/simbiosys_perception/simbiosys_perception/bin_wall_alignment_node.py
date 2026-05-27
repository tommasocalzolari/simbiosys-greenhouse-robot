import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from simbiosys_interfaces.msg import BinWallAlignment

from .bin_wall_fit import fit_bin_wall, scan_to_ordered_points


class BinWallAlignmentNode(Node):
    """Fast lidar estimator for strafe-along-bin control."""

    def __init__(self) -> None:
        super().__init__("bin_wall_alignment_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter(
            "alignment_topic",
            "simbiosys/bin_wall_alignment",
        )
        self.declare_parameter("bed_id", "")
        self.declare_parameter("side", "")
        self.declare_parameter("strafe_direction", "left")
        self.declare_parameter("target_distance_m", 0.35)
        self.declare_parameter("min_range_m", 0.12)
        self.declare_parameter("max_range_m", 2.0)
        self.declare_parameter("scan_angle_offset_rad", -math.pi / 2.0)
        self.declare_parameter("roi_min_angle_rad", math.radians(-60.0))
        self.declare_parameter("roi_max_angle_rad", math.radians(60.0))
        self.declare_parameter("desired_surface_angle_rad", math.pi / 2.0)
        self.declare_parameter("cluster_jump_m", 0.05)
        self.declare_parameter("max_fit_error_m", 0.025)
        self.declare_parameter("min_inliers", 10)
        self.declare_parameter("min_wall_length_m", 0.30)
        self.declare_parameter("corner_endpoint_threshold_m", 0.10)
        self.declare_parameter("max_yaw_error_rad", math.radians(20.0))
        self.declare_parameter("publish_rate_hz", 20.0)

        self._latest_scan: LaserScan | None = None
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(
            BinWallAlignment,
            self._string_parameter("alignment_topic"),
            10,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
            10,
        )

        period = 1.0 / max(1.0, self._double_parameter("publish_rate_hz"))
        self._timer = self.create_timer(period, self._publish_alignment)
        self.get_logger().info(
            "Bin wall alignment node started: "
            f"scan={self._string_parameter('scan_topic')}, "
            f"alignment={self._string_parameter('alignment_topic')}"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    def _publish_alignment(self) -> None:
        with self._lock:
            scan = self._latest_scan

        msg = BinWallAlignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = scan.header.frame_id if scan is not None else ""
        msg.bed_id = self._string_parameter("bed_id")
        msg.side = self._string_parameter("side")
        msg.target_distance_m = self._double_parameter("target_distance_m")

        if scan is None:
            msg.valid = False
            msg.distance_m = math.nan
            msg.distance_error_m = math.nan
            msg.yaw_error_rad = math.nan
            msg.confidence = 0.0
            msg.wall_start_m = math.nan
            msg.wall_end_m = math.nan
            msg.endpoint_in_direction_m = math.nan
            msg.message = "waiting for LaserScan"
            self._publisher.publish(msg)
            return

        points = scan_to_ordered_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            self._double_parameter("min_range_m"),
            self._double_parameter("max_range_m"),
            self._double_parameter("roi_min_angle_rad"),
            self._double_parameter("roi_max_angle_rad"),
            self._double_parameter("scan_angle_offset_rad"),
        )
        result = fit_bin_wall(
            points,
            self._double_parameter("desired_surface_angle_rad"),
            self._string_parameter("strafe_direction"),
            self._double_parameter("max_fit_error_m"),
            self._int_parameter("min_inliers"),
            self._double_parameter("min_wall_length_m"),
            self._double_parameter("cluster_jump_m"),
            self._double_parameter("corner_endpoint_threshold_m"),
            self._double_parameter("max_yaw_error_rad"),
        )

        msg.valid = result.valid
        msg.corner_detected = result.corner_detected
        msg.distance_m = float(result.distance_m)
        msg.distance_error_m = float(result.distance_m - msg.target_distance_m)
        msg.yaw_error_rad = float(result.yaw_error_rad)
        msg.confidence = float(result.confidence)
        msg.wall_length_m = float(result.wall_length_m)
        msg.wall_start_m = float(result.wall_start_m)
        msg.wall_end_m = float(result.wall_end_m)
        msg.endpoint_in_direction_m = float(result.endpoint_in_direction_m)
        msg.message = (
            f"{result.message}; points={len(points)}, "
            f"corner={str(result.corner_detected).lower()}"
        )
        self._publisher.publish(msg)

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BinWallAlignmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

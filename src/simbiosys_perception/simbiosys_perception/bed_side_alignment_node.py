import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from simbiosys_interfaces.msg import BedSideAlignment

from .surface_fit import fit_surface_alignment, scan_ranges_to_points


SURFACE_REGIONS = {
    "front": (-math.radians(45.0), math.radians(45.0), math.pi / 2.0),
    "left": (math.pi / 4.0, 3.0 * math.pi / 4.0, 0.0),
    "right": (-3.0 * math.pi / 4.0, -math.pi / 4.0, 0.0),
}


class BedSideAlignmentNode(Node):
    """Estimate distance/yaw alignment to a nearby planar surface from LaserScan."""

    def __init__(self) -> None:
        super().__init__("bed_side_alignment_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("alignment_topic", "simbiosys/bed_side_alignment")
        self.declare_parameter("bed_id", "")
        self.declare_parameter("side", "")
        self.declare_parameter("surface_region", "front")
        self.declare_parameter("target_distance_m", 0.35)
        self.declare_parameter("min_range_m", 0.15)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("roi_min_angle_rad", math.nan)
        self.declare_parameter("roi_max_angle_rad", math.nan)
        self.declare_parameter("desired_surface_angle_rad", math.nan)
        self.declare_parameter("scan_angle_offset_deg", 90.0)
        self.declare_parameter("max_fit_error_m", 0.05)
        self.declare_parameter("min_inliers", 8)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._latest_scan: LaserScan | None = None
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(
            BedSideAlignment,
            self._string_parameter("alignment_topic"),
            10,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
            10,
        )

        period = 1.0 / max(0.1, self._double_parameter("publish_rate_hz"))
        self._timer = self.create_timer(period, self._publish_alignment)
        self.get_logger().info(
            "Bed-side alignment node started: "
            f"scan={self._string_parameter('scan_topic')}, "
            f"alignment={self._string_parameter('alignment_topic')}"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    def _publish_alignment(self) -> None:
        with self._lock:
            scan = self._latest_scan

        msg = BedSideAlignment()
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
            msg.message = "waiting for LaserScan"
            self._publisher.publish(msg)
            return

        roi_min, roi_max, desired_angle = self._surface_geometry()
        points = scan_ranges_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            self._double_parameter("min_range_m"),
            self._double_parameter("max_range_m"),
            roi_min,
            roi_max,
            self._scan_angle_offset_rad(),
        )
        result = fit_surface_alignment(
            points,
            desired_angle,
            self._double_parameter("max_fit_error_m"),
            self._int_parameter("min_inliers"),
        )

        msg.valid = result.valid
        msg.distance_m = float(result.distance_m)
        msg.distance_error_m = float(result.distance_m - msg.target_distance_m)
        msg.yaw_error_rad = float(result.yaw_error_rad)
        msg.confidence = float(result.confidence)
        msg.message = self._format_result_message(result.message, len(points), roi_min, roi_max)
        self._publisher.publish(msg)

    def _surface_geometry(self) -> tuple[float, float, float]:
        region = self._string_parameter("surface_region").strip().lower()
        roi_min, roi_max, desired_angle = SURFACE_REGIONS.get(region, SURFACE_REGIONS["front"])

        configured_roi_min = self._double_parameter("roi_min_angle_rad")
        configured_roi_max = self._double_parameter("roi_max_angle_rad")
        configured_desired_angle = self._double_parameter("desired_surface_angle_rad")
        if math.isfinite(configured_roi_min):
            roi_min = configured_roi_min
        if math.isfinite(configured_roi_max):
            roi_max = configured_roi_max
        if math.isfinite(configured_desired_angle):
            desired_angle = configured_desired_angle
        return roi_min, roi_max, desired_angle

    def _format_result_message(
        self,
        result_message: str,
        point_count: int,
        roi_min: float,
        roi_max: float,
    ) -> str:
        return (
            f"{result_message}; roi=[{roi_min:.2f}, {roi_max:.2f}]rad, "
            f"scan_offset={self._scan_angle_offset_rad():.2f}rad, "
            f"range=[{self._double_parameter('min_range_m'):.2f}, "
            f"{self._double_parameter('max_range_m'):.2f}]m, points={point_count}"
        )

    def _scan_angle_offset_rad(self) -> float:
        return math.radians(self._double_parameter("scan_angle_offset_deg"))

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BedSideAlignmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

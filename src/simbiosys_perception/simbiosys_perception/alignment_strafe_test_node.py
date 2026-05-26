import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from simbiosys_interfaces.msg import BedSideAlignment


class AlignmentStrafeTestNode(Node):
    """Small guarded test node for checking whether lateral Twist strafing works."""

    def __init__(self) -> None:
        super().__init__("alignment_strafe_test_node")
        self.declare_parameter("alignment_topic", "simbiosys/bed_side_alignment")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel_unstamped")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("strafe_direction", "left")
        self.declare_parameter("strafe_speed_mps", 0.25)
        self.declare_parameter("distance_gain", 0.35)
        self.declare_parameter("yaw_gain", 1.2)
        self.declare_parameter("distance_tolerance_m", 0.01)
        self.declare_parameter("yaw_tolerance_rad", math.radians(1.0))
        self.declare_parameter("strafe_distance_tolerance_m", 0.10)
        self.declare_parameter("strafe_yaw_tolerance_rad", math.radians(10.0))
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("max_forward_speed_mps", 0.5)
        self.declare_parameter("max_angular_speed_radps", 1.0)
        self.declare_parameter("alignment_timeout_sec", 0.5)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("min_side_clearance_m", 0.30)
        self.declare_parameter("side_clearance_min_angle_deg", 45.0)
        self.declare_parameter("side_clearance_max_angle_deg", 135.0)
        self.declare_parameter("control_rate_hz", 10.0)

        self._latest_alignment: BedSideAlignment | None = None
        self._latest_alignment_time = 0.0
        self._latest_scan: LaserScan | None = None
        self._latest_scan_time = 0.0
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            10,
        )
        self.create_subscription(
            BedSideAlignment,
            self._string_parameter("alignment_topic"),
            self._on_alignment,
            10,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
            10,
        )

        period = 1.0 / max(0.1, self._double_parameter("control_rate_hz"))
        self._timer = self.create_timer(period, self._publish_control)
        self.get_logger().info(
            "Alignment strafe test node started with motion disabled. "
            "Set enable_motion:=true only in a safe test area."
        )

    def _on_alignment(self, msg: BedSideAlignment) -> None:
        with self._lock:
            self._latest_alignment = msg
            self._latest_alignment_time = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._latest_scan_time = time.monotonic()

    def _publish_control(self) -> None:
        if not self._bool_parameter("enable_motion"):
            self._publish_zero()
            return

        with self._lock:
            alignment = self._latest_alignment
            alignment_age = time.monotonic() - self._latest_alignment_time
            scan = self._latest_scan
            scan_age = time.monotonic() - self._latest_scan_time

        if alignment is None or alignment_age > self._double_parameter("alignment_timeout_sec"):
            self._publish_zero()
            return
        if not alignment.valid or alignment.confidence < self._double_parameter("min_confidence"):
            self._publish_zero()
            return

        distance_error = float(alignment.distance_error_m)
        if not math.isfinite(distance_error):
            distance_error = float(alignment.target_distance_m - alignment.distance_m)
        yaw_error = float(alignment.yaw_error_rad)

        distance_aligned = math.isfinite(distance_error) and abs(
            distance_error
        ) <= self._double_parameter("strafe_distance_tolerance_m")
        yaw_aligned = math.isfinite(yaw_error) and abs(yaw_error) <= self._double_parameter(
            "strafe_yaw_tolerance_rad"
        )

        twist = Twist()
        side_clear = self._side_clearance_is_safe(scan, scan_age)
        if distance_aligned and yaw_aligned and side_clear:
            twist.linear.y = self._strafe_sign() * abs(self._double_parameter("strafe_speed_mps"))

        if math.isfinite(distance_error) and abs(distance_error) > self._double_parameter(
            "distance_tolerance_m"
        ):
            twist.linear.x = self._clamp(
                self._double_parameter("distance_gain") * distance_error,
                -self._double_parameter("max_forward_speed_mps"),
                self._double_parameter("max_forward_speed_mps"),
            )

        if math.isfinite(yaw_error) and abs(yaw_error) > self._double_parameter(
            "yaw_tolerance_rad"
        ):
            twist.angular.z = self._clamp(
                self._double_parameter("yaw_gain") * yaw_error,
                -self._double_parameter("max_angular_speed_radps"),
                self._double_parameter("max_angular_speed_radps"),
            )

        self._publisher.publish(twist)

    def _publish_zero(self) -> None:
        self._publisher.publish(Twist())

    def _strafe_sign(self) -> float:
        direction = self._string_parameter("strafe_direction").strip().lower()
        return -1.0 if direction in {"right", "negative", "-1"} else 1.0

    def _side_clearance_is_safe(self, scan: LaserScan | None, scan_age: float) -> bool:
        if scan is None or scan_age > self._double_parameter("scan_timeout_sec"):
            return False

        min_angle = math.radians(self._double_parameter("side_clearance_min_angle_deg"))
        max_angle = math.radians(self._double_parameter("side_clearance_max_angle_deg"))
        if self._strafe_sign() < 0.0:
            min_angle, max_angle = -max_angle, -min_angle

        nearest = self._nearest_scan_range(scan, min_angle, max_angle)
        return nearest is None or nearest >= self._double_parameter("min_side_clearance_m")

    @staticmethod
    def _nearest_scan_range(
        scan: LaserScan,
        min_angle: float,
        max_angle: float,
    ) -> float | None:
        nearest: float | None = None
        range_min = max(0.0, float(scan.range_min))
        range_max = float(scan.range_max)
        for index, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m) or range_m < range_min or range_m > range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if angle < min_angle or angle > max_angle:
                continue
            if nearest is None or range_m < nearest:
                nearest = float(range_m)
        return nearest

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignmentStrafeTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

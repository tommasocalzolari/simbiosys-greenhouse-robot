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

    def __init__(self, node_name: str = "alignment_strafe_test_node") -> None:
        super().__init__(node_name)
        self.declare_parameter("alignment_topic", "simbiosys/bed_side_alignment")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("cmd_vel_queue_depth", 1)
        self.declare_parameter("input_queue_depth", 1)
        self.declare_parameter("enable_motion", True)
        self.declare_parameter("strafe_direction", "right")
        self.declare_parameter("strafe_speed_mps", 0.25)
        self.declare_parameter("distance_gain", 1.0) #0.2 for sim
        self.declare_parameter("yaw_gain", 3.0)#1.0 for sim
        self.declare_parameter("distance_tolerance_m", 0.01)
        self.declare_parameter("yaw_tolerance_rad", math.radians(1.0))
        self.declare_parameter("strafe_distance_tolerance_m", 0.10)
        self.declare_parameter("strafe_yaw_tolerance_rad", math.radians(5.0))
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("max_forward_speed_mps", 0.5)
        self.declare_parameter("max_angular_speed_radps", 2.0)
        self.declare_parameter("min_forward_speed_mps", 0.0)
        self.declare_parameter("min_angular_speed_radps", 0.0)
        self.declare_parameter("alignment_timeout_sec", 0.5)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("scan_angle_offset_deg", 90.0)
        self.declare_parameter("min_side_clearance_m", 0.5)
        self.declare_parameter("side_clearance_ignore_below_m", 0.02)
        self.declare_parameter("invert_side_clearance_side", False)
        self.declare_parameter("side_clearance_min_angle_deg", 75.0)
        self.declare_parameter("side_clearance_max_angle_deg", 105.0)
        self.declare_parameter("control_rate_hz", 10.0)

        self._latest_alignment: BedSideAlignment | None = None
        self._latest_alignment_time = 0.0
        self._latest_scan: LaserScan | None = None
        self._latest_scan_time = 0.0
        self._lock = threading.Lock()
        cmd_vel_queue_depth = max(1, self._int_parameter("cmd_vel_queue_depth"))
        input_queue_depth = max(1, self._int_parameter("input_queue_depth"))
        self._publisher = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            cmd_vel_queue_depth,
        )
        self.create_subscription(
            BedSideAlignment,
            self._string_parameter("alignment_topic"),
            self._on_alignment,
            input_queue_depth,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
            input_queue_depth,
        )

        period = 1.0 / max(0.1, self._double_parameter("control_rate_hz"))
        self._timer = self.create_timer(period, self._publish_control)
        self.get_logger().info(
            "Alignment strafe test node started with motion enabled by default. "
            "Set enable_motion:=false to disable movement."
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
            # enforce minimum forward/backward speed so small commands actually move the robot
            min_forward = float(self._double_parameter("min_forward_speed_mps"))
            if math.isfinite(twist.linear.x) and abs(twist.linear.x) > 0.0 and abs(twist.linear.x) < min_forward:
                twist.linear.x = math.copysign(min_forward, twist.linear.x)
                twist.linear.x = self._clamp(
                    twist.linear.x,
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
            # enforce minimum angular speed so small rotation commands actually move the robot
            min_angular = float(self._double_parameter("min_angular_speed_radps"))
            if math.isfinite(twist.angular.z) and abs(twist.angular.z) > 0.0 and abs(twist.angular.z) < min_angular:
                twist.angular.z = math.copysign(min_angular, twist.angular.z)
                twist.angular.z = self._clamp(
                    twist.angular.z,
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
        side_sign = self._strafe_sign()
        if self._bool_parameter("invert_side_clearance_side"):
            side_sign *= -1.0
        if side_sign < 0.0:
            min_angle, max_angle = -max_angle, -min_angle

        nearest = self._nearest_scan_range(
            scan,
            min_angle,
            max_angle,
            self._scan_angle_offset_rad(),
            self._double_parameter("side_clearance_ignore_below_m"),
        )
        return nearest is None or nearest >= self._double_parameter("min_side_clearance_m")

    @staticmethod
    def _nearest_scan_range(
        scan: LaserScan,
        min_angle: float,
        max_angle: float,
        angle_offset_rad: float,
        ignore_below_m: float,
    ) -> float | None:
        nearest: float | None = None
        ignore_below_m = max(0.0, float(ignore_below_m))
        range_max = float(scan.range_max)
        for index, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m) or range_m <= ignore_below_m or range_m > range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment + angle_offset_rad
            if angle < min_angle or angle > max_angle:
                continue
            if nearest is None or range_m < nearest:
                nearest = float(range_m)
        return nearest

    def _scan_angle_offset_rad(self) -> float:
        return math.radians(self._double_parameter("scan_angle_offset_deg"))

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

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

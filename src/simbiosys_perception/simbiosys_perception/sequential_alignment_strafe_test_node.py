import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException

from .alignment_strafe_test_node import AlignmentStrafeTestNode


class SequentialAlignmentStrafeTestNode(AlignmentStrafeTestNode):
    """Align first, then strafe without mixing the two commands."""

    def __init__(self) -> None:
        super().__init__("sequential_alignment_strafe_test_node")
        self.get_logger().info(
            "Sequential alignment strafe node started. It aligns until inside "
            "the strafe gates, then strafes without distance/yaw correction."
        )

    def _publish_control(self) -> None:
        if not self._bool_parameter("enable_motion"):
            self._publish_zero()
            return

        with self._lock:
            alignment = self._latest_alignment
            alignment_age = self._alignment_age()
            scan = self._latest_scan
            scan_age = self._scan_age()

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

        distance_ready = math.isfinite(distance_error) and abs(
            distance_error
        ) <= self._double_parameter("strafe_distance_tolerance_m")
        yaw_ready = math.isfinite(yaw_error) and abs(yaw_error) <= self._double_parameter(
            "strafe_yaw_tolerance_rad"
        )
        side_clear = self._side_clearance_is_safe(scan, scan_age)

        twist = Twist()
        if distance_ready and yaw_ready and side_clear:
            twist.linear.y = self._strafe_sign() * abs(self._double_parameter("strafe_speed_mps"))
            self._publisher.publish(twist)
            return

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

    def _alignment_age(self) -> float:
        import time

        return time.monotonic() - self._latest_alignment_time

    def _scan_age(self) -> float:
        import time

        return time.monotonic() - self._latest_scan_time


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SequentialAlignmentStrafeTestNode()
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

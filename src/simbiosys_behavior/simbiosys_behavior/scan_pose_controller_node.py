import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from simbiosys_interfaces.action import ExecuteScanPosition
from simbiosys_interfaces.msg import BedSideAlignment


class ScanPoseControllerNode(Node):
    """Micro-align the base at one discrete scan position."""

    def __init__(self) -> None:
        super().__init__("scan_pose_controller_node")
        self._callback_group = ReentrantCallbackGroup()
        self._latest_alignment: BedSideAlignment | None = None
        self._latest_alignment_time = 0.0
        self._latest_alignment_seq = 0
        self._filtered_distance_error: float | None = None
        self._filtered_yaw_error: float | None = None
        self._last_control_twist: Twist | None = None
        self._last_control_twist_time = 0.0
        self._lock = threading.Lock()

        self.declare_parameter("alignment_topic", "simbiosys/bed_side_alignment")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("cmd_vel_queue_depth", 1)
        self.declare_parameter("input_queue_depth", 1)
        self.declare_parameter("enable_motion", True)
        self.declare_parameter("target_distance_m", 0.35)
        self.declare_parameter("hold_duration_sec", 1.0)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("max_control_steps", 300)
        self.declare_parameter("alignment_timeout_sec", 0.5)
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("distance_tolerance_m", 0.01)
        self.declare_parameter("yaw_tolerance_rad", math.radians(1.0))
        self.declare_parameter("stable_duration_sec", 0.5)
        self.declare_parameter("distance_gain", 1.0)
        self.declare_parameter("yaw_gain", 3.0)
        self.declare_parameter("max_forward_speed_mps", 0.5)
        self.declare_parameter("max_angular_speed_radps", 2.0)
        self.declare_parameter("min_forward_speed_mps", 0.0)
        self.declare_parameter("min_angular_speed_radps", 0.0)
        self.declare_parameter("alignment_filter_alpha", 0.35)
        self.declare_parameter("require_fresh_alignment_for_control", True)
        self.declare_parameter("control_twist_hold_sec", 0.45)

        cmd_vel_queue_depth = max(1, self._int_parameter("cmd_vel_queue_depth"))
        input_queue_depth = max(1, self._int_parameter("input_queue_depth"))
        self._cmd_vel_publisher = self.create_publisher(
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

        self._action_server = ActionServer(
            self,
            ExecuteScanPosition,
            "simbiosys/execute_scan_position",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "Scan-pose controller ready. It aligns distance/yaw and holds still "
            "at one configured scan position."
        )

    def _on_alignment(self, msg: BedSideAlignment) -> None:
        with self._lock:
            self._latest_alignment = msg
            self._latest_alignment_time = time.monotonic()
            self._latest_alignment_seq += 1

    def _goal_callback(self, goal_request: ExecuteScanPosition.Goal) -> GoalResponse:
        scan_position = goal_request.scan_position
        if not scan_position.scan_position_id.strip():
            self.get_logger().warning("Rejecting scan position without id")
            return GoalResponse.REJECT
        if not scan_position.bed_id.strip():
            self.get_logger().warning("Rejecting scan position without bed_id")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._publish_zero_twist()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        scan_position = goal.scan_position
        bed_id = scan_position.bed_id.strip()
        side = goal.side.strip().lower()
        target_distance = (
            float(goal.target_distance_m)
            if goal.target_distance_m > 0.0
            else self._double_parameter("target_distance_m")
        )
        hold_duration = (
            float(goal.hold_duration_sec)
            if goal.hold_duration_sec > 0.0
            else self._double_parameter("hold_duration_sec")
        )

        if goal.dry_run:
            return self._finish(
                goal_handle,
                True,
                True,
                f"DRY_RUN: scan pose accepted for {scan_position.scan_position_id}",
            )

        if not self._bool_parameter("enable_motion"):
            return self._finish(
                goal_handle,
                False,
                False,
                "PRECONDITION_FAILED: enable_motion is false",
            )

        self._reset_alignment_filter()
        max_steps = max(1, self._int_parameter("max_control_steps"))
        period = 1.0 / max(0.1, self._double_parameter("control_rate_hz"))
        stable_started_at: float | None = None
        last_control_alignment_seq = -1

        for step_index in range(max_steps):
            if goal_handle.is_cancel_requested:
                self._publish_zero_twist()
                goal_handle.canceled()
                return self._result(False, False, "cancelled")

            alignment, alignment_age, alignment_seq = self._latest_alignment_snapshot(
                bed_id,
                side,
            )
            if not self._alignment_is_usable(alignment, alignment_age):
                stable_started_at = None
                self._publish_zero_twist()
                self._publish_feedback(
                    goal_handle,
                    "waiting_for_alignment",
                    step_index / float(max_steps),
                    alignment,
                    "waiting for fresh usable bed-side alignment",
                )
                time.sleep(period)
                continue

            assert alignment is not None
            if (
                self._bool_parameter("require_fresh_alignment_for_control")
                and alignment_seq == last_control_alignment_seq
            ):
                self._publish_held_control_twist()
                time.sleep(period)
                continue

            aligned = self._publish_control_twist(alignment, target_distance)
            last_control_alignment_seq = alignment_seq
            if aligned:
                self._publish_zero_twist()
                if stable_started_at is None:
                    stable_started_at = time.monotonic()
                stable_elapsed = time.monotonic() - stable_started_at
                self._publish_feedback(
                    goal_handle,
                    "aligned",
                    min(0.8, stable_elapsed / max(0.01, self._double_parameter("stable_duration_sec")) * 0.8),
                    alignment,
                    "alignment stable",
                )
                if stable_elapsed >= self._double_parameter("stable_duration_sec"):
                    return self._hold_position(goal_handle, hold_duration, alignment)
            else:
                stable_started_at = None
                self._publish_feedback(
                    goal_handle,
                    "aligning",
                    min(0.75, step_index / float(max_steps)),
                    alignment,
                    "aligning to bed side",
                )

            time.sleep(period)

        return self._finish(
            goal_handle,
            False,
            False,
            f"TIMEOUT: scan pose alignment exceeded max_control_steps={max_steps}",
        )

    def _hold_position(
        self,
        goal_handle,
        hold_duration: float,
        alignment: BedSideAlignment,
    ):
        started_at = time.monotonic()
        period = 1.0 / max(0.1, self._double_parameter("control_rate_hz"))
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_zero_twist()
                goal_handle.canceled()
                return self._result(False, True, "cancelled")
            elapsed = time.monotonic() - started_at
            if elapsed >= hold_duration:
                return self._finish(
                    goal_handle,
                    True,
                    True,
                    "scan pose aligned and held",
                )
            self._publish_zero_twist()
            self._publish_feedback(
                goal_handle,
                "holding",
                0.8 + min(0.2, elapsed / max(0.01, hold_duration) * 0.2),
                alignment,
                "holding aligned scan pose",
            )
            time.sleep(period)

        return self._finish(goal_handle, False, True, "ROS shutdown")

    def _latest_alignment_snapshot(
        self,
        bed_id: str,
        side: str,
    ) -> tuple[BedSideAlignment | None, float, int]:
        with self._lock:
            alignment = self._latest_alignment
            alignment_time = self._latest_alignment_time
            alignment_seq = self._latest_alignment_seq
        alignment_age = time.monotonic() - alignment_time
        if alignment is None:
            return None, alignment_age, alignment_seq
        if alignment.bed_id and alignment.bed_id != bed_id:
            return None, alignment_age, alignment_seq
        if side and alignment.side and alignment.side.lower() != side:
            return None, alignment_age, alignment_seq
        return alignment, alignment_age, alignment_seq

    def _alignment_is_usable(
        self,
        alignment: BedSideAlignment | None,
        alignment_age: float,
    ) -> bool:
        return (
            alignment is not None
            and alignment.valid
            and alignment_age <= self._double_parameter("alignment_timeout_sec")
            and alignment.confidence >= self._double_parameter("min_confidence")
        )

    def _publish_control_twist(
        self,
        alignment: BedSideAlignment,
        target_distance: float,
    ) -> bool:
        distance_error = float(alignment.distance_m - target_distance)
        if math.isfinite(float(alignment.distance_error_m)):
            distance_error = float(alignment.distance_error_m)
        yaw_error = float(alignment.yaw_error_rad)
        distance_error = self._filtered_error("distance", distance_error)
        yaw_error = self._filtered_error("yaw", yaw_error)

        distance_ready = math.isfinite(distance_error) and abs(
            distance_error
        ) <= self._double_parameter("distance_tolerance_m")
        yaw_ready = math.isfinite(yaw_error) and abs(yaw_error) <= self._double_parameter(
            "yaw_tolerance_rad"
        )
        if distance_ready and yaw_ready:
            self._publish_zero_twist()
            return True

        twist = Twist()
        if not distance_ready:
            twist.linear.x = self._limited_command(
                self._double_parameter("distance_gain") * distance_error,
                self._double_parameter("min_forward_speed_mps"),
                self._double_parameter("max_forward_speed_mps"),
            )
        if not yaw_ready:
            twist.angular.z = self._limited_command(
                self._double_parameter("yaw_gain") * yaw_error,
                self._double_parameter("min_angular_speed_radps"),
                self._double_parameter("max_angular_speed_radps"),
            )
        self._cmd_vel_publisher.publish(twist)
        self._last_control_twist = twist
        self._last_control_twist_time = time.monotonic()
        return False

    def _limited_command(self, value: float, minimum: float, maximum: float) -> float:
        command = self._clamp(value, -maximum, maximum)
        minimum = max(0.0, float(minimum))
        if math.isfinite(command) and abs(command) > 0.0 and abs(command) < minimum:
            command = math.copysign(minimum, command)
            command = self._clamp(command, -maximum, maximum)
        return command

    def _publish_held_control_twist(self) -> None:
        if self._last_control_twist is None:
            return
        if (
            time.monotonic() - self._last_control_twist_time
            > self._double_parameter("control_twist_hold_sec")
        ):
            self._publish_zero_twist()
            return
        self._cmd_vel_publisher.publish(self._last_control_twist)

    def _reset_alignment_filter(self) -> None:
        self._filtered_distance_error = None
        self._filtered_yaw_error = None
        self._last_control_twist = None
        self._last_control_twist_time = 0.0

    def _filtered_error(self, name: str, value: float) -> float:
        if not math.isfinite(value):
            return value
        alpha = self._clamp(self._double_parameter("alignment_filter_alpha"), 0.0, 1.0)
        if name == "distance":
            previous = self._filtered_distance_error
            filtered = value if previous is None else alpha * value + (1.0 - alpha) * previous
            self._filtered_distance_error = filtered
            return filtered
        previous = self._filtered_yaw_error
        filtered = value if previous is None else alpha * value + (1.0 - alpha) * previous
        self._filtered_yaw_error = filtered
        return filtered

    def _publish_zero_twist(self) -> None:
        self._last_control_twist = None
        self._last_control_twist_time = 0.0
        self._cmd_vel_publisher.publish(Twist())

    def _publish_feedback(
        self,
        goal_handle,
        phase: str,
        progress: float,
        alignment: BedSideAlignment | None,
        message: str,
    ) -> None:
        feedback = ExecuteScanPosition.Feedback()
        feedback.phase = phase
        feedback.progress = max(0.0, min(1.0, float(progress)))
        if alignment is not None:
            feedback.distance_error_m = float(alignment.distance_error_m)
            feedback.yaw_error_rad = float(alignment.yaw_error_rad)
            feedback.confidence = float(alignment.confidence)
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    def _finish(
        self,
        goal_handle,
        success: bool,
        aligned: bool,
        message: str,
    ):
        self._publish_zero_twist()
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(message)
        return self._result(success, aligned, message)

    def _result(self, success: bool, aligned: bool, message: str):
        result = ExecuteScanPosition.Result()
        result.success = bool(success)
        result.aligned = bool(aligned)
        result.message = message
        return result

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
    node = ScanPoseControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

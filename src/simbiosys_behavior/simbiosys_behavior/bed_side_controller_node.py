import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from simbiosys_interfaces.action import ExecuteBedSideScan
from simbiosys_interfaces.msg import BedSideAlignment, ScanProgress


class BedSideControllerNode(Node):
    """Alignment-only bed-side controller.

    The node uses perception-provided distance/yaw errors to place the MIRTE
    base at the correct pose in front of a bed side. It stops when aligned.
    """

    def __init__(self) -> None:
        super().__init__("bed_side_controller_node")
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
        self.declare_parameter("scan_progress_topic", "simbiosys/scan_progress")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        # Queue depths for velocity commands and perception alignment input.
        self.declare_parameter("cmd_vel_queue_depth", 1)
        self.declare_parameter("input_queue_depth", 1)
        self.declare_parameter("enable_motion", True)
        # target/offsets
        self.declare_parameter("target_distance_m", 0.35)
        self.declare_parameter("distance_tolerance_m", 0.01)
        self.declare_parameter("yaw_tolerance_rad", math.radians(1.0))
        # control gains (match naming and defaults from alignment test)
        self.declare_parameter("distance_gain", 1.0)
        self.declare_parameter("yaw_gain", 3.0)
        # speed limits (match naming)
        self.declare_parameter("max_forward_speed_mps", 0.2)
        self.declare_parameter("max_angular_speed_radps", 0.65)
        # minimum enforced speeds (alignment node exposes these)
        self.declare_parameter("min_forward_speed_mps", 0.0)
        self.declare_parameter("min_angular_speed_radps", 0.0)
        # control timing (use rate like alignment node)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("require_fresh_alignment_for_control", True)
        self.declare_parameter("control_twist_hold_sec", 0.45)
        self.declare_parameter("alignment_filter_alpha", 0.35)
        self.declare_parameter("max_control_steps", 600)
        self.declare_parameter("allow_timeout_success", False)
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("alignment_timeout_sec", 0.5)
        self.declare_parameter("end_reached_alignment_lost_cycles", 3)

        cmd_vel_queue_depth = max(1, self._int_parameter("cmd_vel_queue_depth"))
        input_queue_depth = max(1, self._int_parameter("input_queue_depth"))
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            cmd_vel_queue_depth,
        )
        self._scan_progress_publisher = self.create_publisher(
            ScanProgress,
            self._string_parameter("scan_progress_topic"),
            10,
        )
        self.create_subscription(
            BedSideAlignment,
            self._string_parameter("alignment_topic"),
            self._on_alignment,
            input_queue_depth,
        )

        self._action_server = ActionServer(
            self,
            ExecuteBedSideScan,
            "simbiosys/execute_bed_side_scan",
            execute_callback=self._execute_scan,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Bed-side alignment controller ready. It only corrects distance/yaw "
            "and stops; it does not strafe along the bed."
        )

    def _on_alignment(self, msg: BedSideAlignment) -> None:
        with self._lock:
            self._latest_alignment = msg
            self._latest_alignment_time = time.monotonic()
            self._latest_alignment_seq += 1

    def _goal_callback(self, goal_request: ExecuteBedSideScan.Goal) -> GoalResponse:
        if not goal_request.bed_id.strip():
            self.get_logger().warning("Rejecting bed-side alignment without bed_id")
            return GoalResponse.REJECT
        if goal_request.side.strip().lower() not in ("a", "b"):
            self.get_logger().warning(
                f"Rejecting bed-side alignment with invalid side '{goal_request.side}'"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._publish_zero_twist()
        return CancelResponse.ACCEPT

    def _execute_scan(self, goal_handle):
        goal = goal_handle.request
        bed_id = goal.bed_id.strip()
        side = goal.side.strip().lower()
        self._publish_feedback(
            goal_handle,
            "starting",
            0.05,
            0,
            0,
            f"Starting bed-side alignment for {bed_id}:{side}",
        )
        self._publish_scan_progress(
            bed_id,
            side,
            "starting",
            0,
            0,
            f"Starting bed-side alignment for {bed_id}:{side}",
        )

        if goal.dry_run:
            return self._finish(
                goal_handle,
                True,
                0,
                False,
                (
                    f"DRY_RUN: bed-side alignment accepted {bed_id}:{side}; "
                    "no base or arm motion was commanded"
                ),
            )

        if not self._bool_parameter("enable_motion"):
            return self._finish(
                goal_handle,
                False,
                0,
                False,
                "PRECONDITION_FAILED: enable_motion is false",
            )

        alignment, alignment_age, _alignment_seq = self._latest_alignment_snapshot(bed_id, side)
        if not self._alignment_is_usable(alignment, alignment_age):
            return self._finish(
                goal_handle,
                False,
                0,
                False,
                "PRECONDITION_FAILED: no valid bed-side alignment message received",
            )

        max_steps = max(1, self._int_parameter("max_control_steps"))
        period = 1.0 / max(0.1, self._double_parameter("control_rate_hz"))
        alignment_lost_cycles = 0
        last_control_alignment_seq = -1
        self._reset_alignment_filter()

        for step_index in range(max_steps):
            if goal_handle.is_cancel_requested:
                self._publish_zero_twist()
                goal_handle.canceled()
                return self._result(False, 0, False, "cancelled")

            alignment, alignment_age, alignment_seq = self._latest_alignment_snapshot(bed_id, side)
            if not self._alignment_is_usable(alignment, alignment_age):
                alignment_lost_cycles += 1
                self._publish_zero_twist()
                if alignment_lost_cycles < self._int_parameter(
                    "end_reached_alignment_lost_cycles"
                ):
                    time.sleep(period)
                    continue
                return self._finish(
                    goal_handle,
                    False,
                    0,
                    False,
                    "PRECONDITION_FAILED: bed-side alignment became unavailable",
                )

            alignment_lost_cycles = 0
            assert alignment is not None
            if (
                self._bool_parameter("require_fresh_alignment_for_control")
                and alignment_seq == last_control_alignment_seq
            ):
                self._publish_held_control_twist()
                time.sleep(period)
                continue
            phase = self._publish_control_twist(alignment)
            last_control_alignment_seq = alignment_seq
            if phase == "aligned":
                message = f"ALIGNED: corrected distance and yaw for {bed_id}:{side}"
                self._publish_feedback(
                    goal_handle,
                    phase,
                    1.0,
                    0,
                    0,
                    message,
                )
                self._publish_scan_progress(
                    bed_id,
                    side,
                    phase,
                    0,
                    0,
                    message,
                )
                return self._finish(goal_handle, True, 0, False, message)

            progress = min(0.95, (step_index + 1) / float(max_steps))
            self._publish_feedback(
                goal_handle,
                phase,
                progress,
                0,
                0,
                f"{phase} {bed_id}:{side}; correcting distance and yaw",
            )
            time.sleep(period)

        self._publish_zero_twist()

        if not self._bool_parameter("allow_timeout_success"):
            return self._finish(
                goal_handle,
                False,
                0,
                False,
                (
                    f"EXECUTION_TIMEOUT: max_control_steps={max_steps} reached "
                    f"before alignment completed for {bed_id}:{side}"
                ),
            )

        return self._finish(
            goal_handle,
            True,
            0,
            False,
            f"Alignment timeout accepted for {bed_id}:{side}",
        )

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
        if alignment.side and alignment.side.lower() != side:
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

    def _publish_control_twist(self, alignment: BedSideAlignment) -> str:
        twist = Twist()
        distance_error = float(alignment.distance_error_m)
        if not math.isfinite(distance_error):
            distance_error = float(alignment.distance_m - alignment.target_distance_m)
        yaw_error = float(alignment.yaw_error_rad)
        distance_error = self._filtered_error("distance", distance_error)
        yaw_error = self._filtered_error("yaw", yaw_error)
        distance_ready = math.isfinite(distance_error) and abs(
            distance_error
        ) <= self._double_parameter("distance_tolerance_m")
        yaw_ready = math.isfinite(yaw_error) and abs(yaw_error) <= self._double_parameter(
            "yaw_tolerance_rad"
        )

        phase = "aligning"
        if distance_ready and yaw_ready:
            self._publish_zero_twist()
            return "aligned"

        if abs(distance_error) > self._double_parameter("distance_tolerance_m"):
            twist.linear.x = self._clamp(
                self._double_parameter("distance_gain") * distance_error,
                -self._double_parameter("max_forward_speed_mps"),
                self._double_parameter("max_forward_speed_mps"),
            )
            # Enforce minimum speed so small commands actually move the robot.
            min_forward = float(self._double_parameter("min_forward_speed_mps"))
            if (
                math.isfinite(twist.linear.x)
                and abs(twist.linear.x) > 0.0
                and abs(twist.linear.x) < min_forward
            ):
                twist.linear.x = math.copysign(min_forward, twist.linear.x)
                twist.linear.x = self._clamp(
                    twist.linear.x,
                    -self._double_parameter("max_forward_speed_mps"),
                    self._double_parameter("max_forward_speed_mps"),
                )
        if abs(yaw_error) > self._double_parameter("yaw_tolerance_rad"):
            twist.angular.z = self._clamp(
                self._double_parameter("yaw_gain") * yaw_error,
                -self._double_parameter("max_angular_speed_radps"),
                self._double_parameter("max_angular_speed_radps"),
            )
            # Enforce minimum speed so small commands actually move the robot.
            min_angular = float(self._double_parameter("min_angular_speed_radps"))
            if (
                math.isfinite(twist.angular.z)
                and abs(twist.angular.z) > 0.0
                and abs(twist.angular.z) < min_angular
            ):
                twist.angular.z = math.copysign(min_angular, twist.angular.z)
                twist.angular.z = self._clamp(
                    twist.angular.z,
                    -self._double_parameter("max_angular_speed_radps"),
                    self._double_parameter("max_angular_speed_radps"),
                )
        self._cmd_vel_publisher.publish(twist)
        self._last_control_twist = twist
        self._last_control_twist_time = time.monotonic()
        return phase

    def _publish_held_control_twist(self) -> None:
        if self._last_control_twist is None:
            return
        twist_age = time.monotonic() - self._last_control_twist_time
        if twist_age > self._double_parameter("control_twist_hold_sec"):
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

        alpha = self._clamp(
            self._double_parameter("alignment_filter_alpha"),
            0.0,
            1.0,
        )
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
        flowers_detected: int,
        retry_count: int,
        message: str,
    ) -> None:
        feedback = ExecuteBedSideScan.Feedback()
        feedback.phase = phase
        feedback.progress = max(0.0, min(1.0, float(progress)))
        feedback.flowers_detected = int(flowers_detected)
        feedback.retry_count = int(retry_count)
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    def _publish_scan_progress(
        self,
        bed_id: str,
        side: str,
        detection_status: str,
        flowers_detected: int,
        retry_count: int,
        message: str,
        active_flower_id: str = "",
        error: bool = False,
    ) -> None:
        progress = ScanProgress()
        progress.active_bed_id = bed_id
        progress.active_scan_position_id = f"{bed_id}:{side}"
        progress.active_flower_id = active_flower_id
        progress.scan_index = int(flowers_detected)
        progress.scan_total = self._int_parameter("max_control_steps")
        progress.detection_status = detection_status
        progress.retry_count = int(retry_count)
        progress.error = error
        progress.message = message
        self._scan_progress_publisher.publish(progress)

    def _finish(
        self,
        goal_handle,
        success: bool,
        flowers_detected: int,
        retry_used: bool,
        message: str,
    ):
        self._publish_zero_twist()
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(message)
        return self._result(success, flowers_detected, retry_used, message)

    def _result(
        self,
        success: bool,
        flowers_detected: int,
        retry_used: bool,
        message: str,
    ):
        result = ExecuteBedSideScan.Result()
        result.success = bool(success)
        result.flowers_detected = int(flowers_detected)
        result.retry_used = bool(retry_used)
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
    node = BedSideControllerNode()
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

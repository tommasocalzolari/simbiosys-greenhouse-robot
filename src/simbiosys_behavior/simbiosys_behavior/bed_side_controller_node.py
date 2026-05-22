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
from simbiosys_interfaces.msg import BedSideAlignment, FlowerTarget, ScanProgress


class BedSideControllerNode(Node):
    """Dry-run-safe scaffold for bed-side visual/depth servo scanning.

    The node owns the future local control loop that keeps the MIRTE base
    perpendicular to a bed side and adjusts arm height from flower detections.
    It does not interpret images itself; perception publishes typed alignment
    and flower target messages.
    """

    def __init__(self) -> None:
        super().__init__("bed_side_controller_node")
        self._callback_group = ReentrantCallbackGroup()
        self._latest_alignment: BedSideAlignment | None = None
        self._latest_flower: FlowerTarget | None = None
        self._lock = threading.Lock()

        self.declare_parameter("alignment_topic", "simbiosys/bed_side_alignment")
        self.declare_parameter("flower_target_topic", "simbiosys/flower_target")
        self.declare_parameter("scan_progress_topic", "simbiosys/scan_progress")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("target_distance_m", 0.35)
        self.declare_parameter("distance_tolerance_m", 0.03)
        self.declare_parameter("yaw_tolerance_rad", 0.08)
        self.declare_parameter("linear_gain", 0.3)
        self.declare_parameter("angular_gain", 0.8)
        self.declare_parameter("max_linear_speed_mps", 0.05)
        self.declare_parameter("max_angular_speed_radps", 0.15)
        self.declare_parameter("control_period_sec", 0.25)
        self.declare_parameter("max_control_steps", 20)

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            10,
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
            10,
        )
        self.create_subscription(
            FlowerTarget,
            self._string_parameter("flower_target_topic"),
            self._on_flower_target,
            10,
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
            "Bed-side controller ready in dry-run-safe mode. Set enable_motion:=true "
            "only after perception alignment and robot safety are validated."
        )

    def _on_alignment(self, msg: BedSideAlignment) -> None:
        with self._lock:
            self._latest_alignment = msg

    def _on_flower_target(self, msg: FlowerTarget) -> None:
        with self._lock:
            self._latest_flower = msg

    def _goal_callback(self, goal_request: ExecuteBedSideScan.Goal) -> GoalResponse:
        if not goal_request.bed_id.strip():
            self.get_logger().warning("Rejecting bed-side scan without bed_id")
            return GoalResponse.REJECT
        if goal_request.side.strip().lower() not in ("a", "b"):
            self.get_logger().warning(
                f"Rejecting bed-side scan with invalid side '{goal_request.side}'"
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
        min_flowers = max(0, int(goal.min_flower_count))

        self._publish_feedback(
            goal_handle,
            "starting",
            0.05,
            0,
            0,
            f"Starting bed-side scan for {bed_id}:{side}",
        )
        self._publish_scan_progress(
            bed_id,
            side,
            "starting",
            0,
            0,
            f"Starting bed-side scan for {bed_id}:{side}",
        )

        if goal.dry_run:
            return self._finish(
                goal_handle,
                True,
                0,
                False,
                (
                    f"DRY_RUN: bed-side scan infrastructure accepted {bed_id}:{side}; "
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

        alignment = self._latest_alignment_for(bed_id, side)
        if alignment is None:
            return self._finish(
                goal_handle,
                False,
                0,
                False,
                "PRECONDITION_FAILED: no valid bed-side alignment message received",
            )

        flowers_detected = 0
        retry_count = 0
        max_steps = max(1, self._int_parameter("max_control_steps"))
        period = max(0.05, self._double_parameter("control_period_sec"))

        for step_index in range(max_steps):
            if goal_handle.is_cancel_requested:
                self._publish_zero_twist()
                goal_handle.canceled()
                return self._result(False, flowers_detected, retry_count > 0, "cancelled")

            alignment = self._latest_alignment_for(bed_id, side)
            if alignment is None:
                return self._finish(
                    goal_handle,
                    False,
                    flowers_detected,
                    retry_count > 0,
                    "PRECONDITION_FAILED: bed-side alignment became unavailable",
                )

            self._publish_control_twist(alignment)
            flower = self._latest_flower_for(bed_id, side)
            if flower is not None and flower.detected:
                flowers_detected += 1
                phase = "harvest_ready" if flower.ready_for_harvest else "flower_seen"
                message = (
                    f"{phase}: {flower.flower_id or 'runtime_flower'} "
                    f"height={flower.height_cm:.1f}cm"
                )
                self._publish_scan_progress(
                    bed_id,
                    side,
                    phase,
                    flowers_detected,
                    retry_count,
                    message,
                    active_flower_id=flower.flower_id,
                )

            progress = min(0.95, (step_index + 1) / float(max_steps))
            self._publish_feedback(
                goal_handle,
                "scanning",
                progress,
                flowers_detected,
                retry_count,
                f"Scanning {bed_id}:{side}; flowers_detected={flowers_detected}",
            )
            time.sleep(period)

        self._publish_zero_twist()

        if flowers_detected < min_flowers:
            retry_count = 1
            message = (
                f"EXECUTION_FAILED: found {flowers_detected} flower(s), "
                f"minimum for {bed_id}:{side} is {min_flowers}; retry scaffold reached"
            )
            return self._finish(goal_handle, False, flowers_detected, True, message)

        return self._finish(
            goal_handle,
            True,
            flowers_detected,
            retry_count > 0,
            f"Completed bed-side scan for {bed_id}:{side}",
        )

    def _latest_alignment_for(
        self,
        bed_id: str,
        side: str,
    ) -> BedSideAlignment | None:
        with self._lock:
            alignment = self._latest_alignment
        if alignment is None or not alignment.valid:
            return None
        if alignment.bed_id and alignment.bed_id != bed_id:
            return None
        if alignment.side and alignment.side.lower() != side:
            return None
        return alignment

    def _latest_flower_for(self, bed_id: str, side: str) -> FlowerTarget | None:
        with self._lock:
            flower = self._latest_flower
        if flower is None:
            return None
        if flower.bed_id and flower.bed_id != bed_id:
            return None
        if flower.side and flower.side.lower() != side:
            return None
        return flower

    def _publish_control_twist(self, alignment: BedSideAlignment) -> None:
        twist = Twist()
        distance_error = float(alignment.distance_error_m)
        if not math.isfinite(distance_error):
            distance_error = float(alignment.target_distance_m - alignment.distance_m)
        yaw_error = float(alignment.yaw_error_rad)

        if abs(distance_error) > self._double_parameter("distance_tolerance_m"):
            twist.linear.x = self._clamp(
                self._double_parameter("linear_gain") * distance_error,
                -self._double_parameter("max_linear_speed_mps"),
                self._double_parameter("max_linear_speed_mps"),
            )
        if abs(yaw_error) > self._double_parameter("yaw_tolerance_rad"):
            twist.angular.z = self._clamp(
                self._double_parameter("angular_gain") * yaw_error,
                -self._double_parameter("max_angular_speed_radps"),
                self._double_parameter("max_angular_speed_radps"),
            )
        self._cmd_vel_publisher.publish(twist)

    def _publish_zero_twist(self) -> None:
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

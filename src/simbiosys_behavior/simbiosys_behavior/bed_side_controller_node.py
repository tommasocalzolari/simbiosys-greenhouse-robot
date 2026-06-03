import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

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
        self._latest_alignment_time = 0.0
        self._latest_alignment_seq = 0
        self._latest_flower: FlowerTarget | None = None
        self._latest_scan: LaserScan | None = None
        self._latest_scan_time = 0.0
        self._filtered_distance_error: float | None = None
        self._filtered_yaw_error: float | None = None
        self._last_control_twist: Twist | None = None
        self._last_control_twist_time = 0.0
        self._lock = threading.Lock()

        self.declare_parameter("alignment_topic", "simbiosys/bed_side_alignment")
        self.declare_parameter("flower_target_topic", "simbiosys/flower_target")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_progress_topic", "simbiosys/scan_progress")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        # queue depths and input sizes (match alignment_strafe_test_node)
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
        self.declare_parameter("max_forward_speed_mps", 0.5)
        self.declare_parameter("max_angular_speed_radps", 2.0)
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
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("strafe_direction", "right")
        self.declare_parameter("strafe_speed_mps", 0.25)
        self.declare_parameter("strafe_distance_tolerance_m", 0.10)
        self.declare_parameter("strafe_yaw_tolerance_rad", math.radians(5.0))
        self.declare_parameter("min_strafe_time_before_end_sec", 0.5)
        self.declare_parameter("end_reached_gap_cycles", 2)
        self.declare_parameter("enable_blocked_side_end_detection", False)
        self.declare_parameter("end_reached_blocked_cycles", 2)
        self.declare_parameter("end_reached_alignment_lost_cycles", 3)
        self.declare_parameter("corner_gap_min_angle_deg", -45.0)
        self.declare_parameter("corner_gap_max_angle_deg", 45.0)
        self.declare_parameter("corner_gap_range_ratio", 1.8)
        self.declare_parameter("corner_gap_min_delta_m", 0.35)
        self.declare_parameter("corner_gap_min_samples", 4)
        self.declare_parameter("min_side_clearance_m", 0.5)
        self.declare_parameter("side_clearance_blocked_min_samples", 3)
        self.declare_parameter("side_clearance_blocked_min_fraction", 0.10)
        self.declare_parameter("side_clearance_ignore_below_m", 0.02)
        self.declare_parameter("invert_side_clearance_side", False)
        self.declare_parameter("side_clearance_min_angle_deg", 75.0)
        self.declare_parameter("side_clearance_max_angle_deg", 105.0)
        self.declare_parameter("scan_angle_offset_deg", 90.0)

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
        self.create_subscription(
            FlowerTarget,
            self._string_parameter("flower_target_topic"),
            self._on_flower_target,
            input_queue_depth,
        )
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
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
            "Bed-side controller ready in dry-run-safe mode. Set enable_motion:=true "
            "only after perception alignment and robot safety are validated."
        )

    def _on_alignment(self, msg: BedSideAlignment) -> None:
        with self._lock:
            self._latest_alignment = msg
            self._latest_alignment_time = time.monotonic()
            self._latest_alignment_seq += 1

    def _on_flower_target(self, msg: FlowerTarget) -> None:
        with self._lock:
            self._latest_flower = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._latest_scan_time = time.monotonic()

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

        alignment, alignment_age, _alignment_seq = self._latest_alignment_snapshot(bed_id, side)
        if not self._alignment_is_usable(alignment, alignment_age):
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
        period = 1.0 / max(0.1, self._double_parameter("control_rate_hz"))
        corner_gap_cycles = 0
        blocked_cycles = 0
        alignment_lost_cycles = 0
        strafe_started_at: float | None = None
        last_control_alignment_seq = -1
        self._reset_alignment_filter()

        for step_index in range(max_steps):
            if goal_handle.is_cancel_requested:
                self._publish_zero_twist()
                goal_handle.canceled()
                return self._result(False, flowers_detected, retry_count > 0, "cancelled")

            alignment, alignment_age, alignment_seq = self._latest_alignment_snapshot(bed_id, side)
            scan, scan_age = self._latest_scan_snapshot()
            if not self._alignment_is_usable(alignment, alignment_age):
                alignment_lost_cycles += 1
                self._publish_zero_twist()
                if (
                    strafe_started_at is not None
                    and alignment_lost_cycles
                    >= self._int_parameter("end_reached_alignment_lost_cycles")
                ):
                    message = (
                        f"END_REACHED: alignment lost at end of {bed_id}:{side}; "
                        f"flowers_detected={flowers_detected}"
                    )
                    self._publish_feedback(
                        goal_handle,
                        "end_reached",
                        1.0,
                        flowers_detected,
                        retry_count,
                        message,
                    )
                    self._publish_scan_progress(
                        bed_id,
                        side,
                        "end_reached",
                        flowers_detected,
                        retry_count,
                        message,
                    )
                    return self._finish(
                        goal_handle,
                        True,
                        flowers_detected,
                        retry_count > 0,
                        message,
                    )
                if alignment_lost_cycles < self._int_parameter(
                    "end_reached_alignment_lost_cycles"
                ):
                    time.sleep(period)
                    continue
                if strafe_started_at is None:
                    return self._finish(
                        goal_handle,
                        False,
                        flowers_detected,
                        retry_count > 0,
                        "PRECONDITION_FAILED: bed-side alignment became unavailable",
                    )
                time.sleep(period)
                continue

            alignment_lost_cycles = 0
            assert alignment is not None
            if (
                self._bool_parameter("require_fresh_alignment_for_control")
                and alignment_seq == last_control_alignment_seq
            ):
                self._publish_held_control_twist()
                time.sleep(period)
                continue
            side_clearance = self._side_clearance_status(scan, scan_age)
            corner_gap = self._corner_gap_status(scan, scan_age)
            strafe_has_started_long_enough = (
                strafe_started_at is not None
                and time.monotonic() - strafe_started_at
                >= self._double_parameter("min_strafe_time_before_end_sec")
            )
            if strafe_has_started_long_enough and corner_gap:
                corner_gap_cycles += 1
            else:
                corner_gap_cycles = 0

            if corner_gap_cycles >= self._int_parameter("end_reached_gap_cycles"):
                message = (
                    f"END_REACHED: front scan opened toward strafe direction at "
                    f"end of {bed_id}:{side}; flowers_detected={flowers_detected}"
                )
                self._publish_feedback(
                    goal_handle,
                    "end_reached",
                    1.0,
                    flowers_detected,
                    retry_count,
                    message,
                )
                self._publish_scan_progress(
                    bed_id,
                    side,
                    "end_reached",
                    flowers_detected,
                    retry_count,
                    message,
                )
                return self._finish(
                    goal_handle,
                    True,
                    flowers_detected,
                    retry_count > 0,
                    message,
                )

            if (
                strafe_has_started_long_enough
                and side_clearance == "blocked"
            ):
                blocked_cycles += 1
            else:
                blocked_cycles = 0

            if blocked_cycles >= self._int_parameter("end_reached_blocked_cycles"):
                if self._bool_parameter("enable_blocked_side_end_detection"):
                    message = (
                        f"END_REACHED: side clearance blocked at end of {bed_id}:{side}; "
                        f"flowers_detected={flowers_detected}"
                    )
                    self._publish_feedback(
                        goal_handle,
                        "end_reached",
                        1.0,
                        flowers_detected,
                        retry_count,
                        message,
                    )
                    self._publish_scan_progress(
                        bed_id,
                        side,
                        "end_reached",
                        flowers_detected,
                        retry_count,
                        message,
                    )
                    return self._finish(
                        goal_handle,
                        True,
                        flowers_detected,
                        retry_count > 0,
                        message,
                    )
                blocked_cycles = 0

            phase = self._publish_control_twist(alignment, side_clearance)
            last_control_alignment_seq = alignment_seq
            if phase == "strafing" and strafe_started_at is None:
                strafe_started_at = time.monotonic()

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
                phase,
                progress,
                flowers_detected,
                retry_count,
                (
                    f"{phase} {bed_id}:{side}; flowers_detected={flowers_detected}; "
                    f"side_clearance={side_clearance}; corner_gap={corner_gap}"
                ),
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

        if not self._bool_parameter("allow_timeout_success"):
            return self._finish(
                goal_handle,
                False,
                flowers_detected,
                retry_count > 0,
                (
                    f"EXECUTION_TIMEOUT: max_control_steps={max_steps} reached "
                    f"before end of {bed_id}:{side}"
                ),
            )

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
        alignment, alignment_age, _alignment_seq = self._latest_alignment_snapshot(bed_id, side)
        if not self._alignment_is_usable(alignment, alignment_age):
            return None
        return alignment

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

    def _latest_scan_snapshot(self) -> tuple[LaserScan | None, float]:
        with self._lock:
            scan = self._latest_scan
            scan_time = self._latest_scan_time
        return scan, time.monotonic() - scan_time

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

    def _publish_control_twist(self, alignment: BedSideAlignment, side_clearance: str) -> str:
        twist = Twist()
        distance_error = float(alignment.distance_error_m)
        if not math.isfinite(distance_error):
            distance_error = float(alignment.target_distance_m - alignment.distance_m)
        yaw_error = float(alignment.yaw_error_rad)
        distance_error = self._filtered_error("distance", distance_error)
        yaw_error = self._filtered_error("yaw", yaw_error)
        distance_ready = math.isfinite(distance_error) and abs(
            distance_error
        ) <= self._double_parameter("strafe_distance_tolerance_m")
        yaw_ready = math.isfinite(yaw_error) and abs(yaw_error) <= self._double_parameter(
            "strafe_yaw_tolerance_rad"
        )

        phase = "aligning"
        if distance_ready and yaw_ready and side_clearance == "safe":
            twist.linear.y = self._strafe_sign() * abs(
                self._double_parameter("strafe_speed_mps")
            )
            phase = "strafing"

        if abs(distance_error) > self._double_parameter("distance_tolerance_m"):
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
        if abs(yaw_error) > self._double_parameter("yaw_tolerance_rad"):
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

    def _side_clearance_status(self, scan: LaserScan | None, scan_age: float) -> str:
        if scan is None or scan_age > self._double_parameter("scan_timeout_sec"):
            return "unknown"

        min_angle = math.radians(self._double_parameter("side_clearance_min_angle_deg"))
        max_angle = math.radians(self._double_parameter("side_clearance_max_angle_deg"))
        side_sign = self._strafe_sign()
        if self._bool_parameter("invert_side_clearance_side"):
            side_sign *= -1.0
        if side_sign < 0.0:
            min_angle, max_angle = -max_angle, -min_angle

        ranges = self._scan_ranges_in_sector(
            scan,
            min_angle,
            max_angle,
            self._scan_angle_offset_rad(),
            self._double_parameter("side_clearance_ignore_below_m"),
        )
        if not ranges:
            return "safe"

        blocked_ranges = [
            range_m
            for range_m in ranges
            if range_m < self._double_parameter("min_side_clearance_m")
        ]
        min_blocked_samples = max(
            1,
            self._int_parameter("side_clearance_blocked_min_samples"),
        )
        min_blocked_fraction = self._clamp(
            self._double_parameter("side_clearance_blocked_min_fraction"),
            0.0,
            1.0,
        )
        if (
            len(blocked_ranges) >= min_blocked_samples
            and len(blocked_ranges) / float(len(ranges)) >= min_blocked_fraction
        ):
            return "blocked"
        return "safe"

    def _corner_gap_status(self, scan: LaserScan | None, scan_age: float) -> bool:
        if scan is None or scan_age > self._double_parameter("scan_timeout_sec"):
            return False

        min_angle = math.radians(self._double_parameter("corner_gap_min_angle_deg"))
        max_angle = math.radians(self._double_parameter("corner_gap_max_angle_deg"))
        if min_angle >= max_angle:
            return False

        if self._strafe_sign() < 0.0:
            strafe_min, strafe_max = min_angle, 0.0
            other_min, other_max = 0.0, max_angle
        else:
            strafe_min, strafe_max = 0.0, max_angle
            other_min, other_max = min_angle, 0.0

        ignore_below = self._double_parameter("side_clearance_ignore_below_m")
        angle_offset = self._scan_angle_offset_rad()
        strafe_ranges = self._scan_ranges_in_sector(
            scan,
            strafe_min,
            strafe_max,
            angle_offset,
            ignore_below,
        )
        other_ranges = self._scan_ranges_in_sector(
            scan,
            other_min,
            other_max,
            angle_offset,
            ignore_below,
        )
        min_samples = self._int_parameter("corner_gap_min_samples")
        if len(strafe_ranges) < min_samples or len(other_ranges) < min_samples:
            return False

        strafe_median = self._median(strafe_ranges)
        other_median = self._median(other_ranges)
        return (
            strafe_median - other_median
            >= self._double_parameter("corner_gap_min_delta_m")
            and strafe_median
            >= other_median * self._double_parameter("corner_gap_range_ratio")
        )

    def _strafe_sign(self) -> float:
        direction = self._string_parameter("strafe_direction").strip().lower()
        return -1.0 if direction in {"right", "negative", "-1"} else 1.0

    @staticmethod
    def _scan_ranges_in_sector(
        scan: LaserScan,
        min_angle: float,
        max_angle: float,
        angle_offset_rad: float,
        ignore_below_m: float,
    ) -> list[float]:
        ranges: list[float] = []
        ignore_below_m = max(0.0, float(ignore_below_m))
        range_max = float(scan.range_max)
        for index, range_m in enumerate(scan.ranges):
            if not math.isfinite(range_m):
                if range_max <= 0.0:
                    continue
                range_m = range_max
            if range_m <= ignore_below_m or range_m > range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment + angle_offset_rad
            if min_angle <= angle <= max_angle:
                ranges.append(float(range_m))
        return ranges

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

    @staticmethod
    def _median(values: list[float]) -> float:
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    def _scan_angle_offset_rad(self) -> float:
        return math.radians(self._double_parameter("scan_angle_offset_deg"))

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

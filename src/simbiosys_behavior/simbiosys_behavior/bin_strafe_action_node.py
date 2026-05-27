import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from simbiosys_interfaces.action import ExecuteBinStrafe
from simbiosys_interfaces.msg import BinWallAlignment


class BinStrafeActionNode(Node):
    """Strafe along a detected bin wall until the corner."""

    def __init__(self) -> None:
        super().__init__("bin_strafe_action_node")
        self._callback_group = ReentrantCallbackGroup()
        self._latest_alignment_msg: BinWallAlignment | None = None
        self._latest_alignment_time = 0.0
        self._lock = threading.Lock()

        self.declare_parameter(
            "alignment_topic",
            "simbiosys/bin_wall_alignment",
        )
        self.declare_parameter(
            "cmd_vel_topic",
            "/mirte_base_controller/cmd_vel",
        )
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("default_target_distance_m", 0.35)
        self.declare_parameter("default_strafe_speed_mps", 0.25)
        self.declare_parameter("default_timeout_sec", 20.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("alignment_timeout_sec", 0.25)
        self.declare_parameter("min_confidence", 0.45)
        self.declare_parameter("distance_tolerance_m", 0.015)
        self.declare_parameter("yaw_tolerance_rad", math.radians(1.5))
        self.declare_parameter("strafe_distance_tolerance_m", 0.04)
        self.declare_parameter("strafe_yaw_tolerance_rad", math.radians(5.0))
        self.declare_parameter("distance_gain", 0.3)
        self.declare_parameter("yaw_gain", 2.5)
        self.declare_parameter("max_strafe_speed_mps", 0.5)
        self.declare_parameter("max_distance_speed_mps", 0.25)
        self.declare_parameter("max_angular_speed_radps", 0.3)
        self.declare_parameter("corner_confirmations", 3)
        self.declare_parameter("lost_wall_timeout_sec", 0.75)

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            10,
        )
        self.create_subscription(
            BinWallAlignment,
            self._string_parameter("alignment_topic"),
            self._on_alignment,
            10,
        )
        self._action_server = ActionServer(
            self,
            ExecuteBinStrafe,
            "simbiosys/execute_bin_strafe",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "Bin strafe action server ready. Set enable_motion:=true "
            "after lidar alignment is validated."
        )

    def _on_alignment(self, msg: BinWallAlignment) -> None:
        with self._lock:
            self._latest_alignment_msg = msg
            self._latest_alignment_time = time.monotonic()

    def _goal_callback(
        self,
        goal_request: ExecuteBinStrafe.Goal,
    ) -> GoalResponse:
        direction = goal_request.direction.strip().lower()
        if direction not in {"left", "right"}:
            self.get_logger().warning(
                f"Rejecting bin strafe direction '{goal_request.direction}'"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._publish_zero_twist()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        direction = goal.direction.strip().lower()
        target_distance = (
            float(goal.target_distance_m)
            if goal.target_distance_m > 0.0
            else self._double_parameter("default_target_distance_m")
        )
        strafe_speed = (
            abs(float(goal.strafe_speed_mps))
            if goal.strafe_speed_mps > 0.0
            else self._double_parameter("default_strafe_speed_mps")
        )
        timeout_sec = (
            float(goal.timeout_sec)
            if goal.timeout_sec > 0.0
            else self._double_parameter("default_timeout_sec")
        )

        if goal.dry_run:
            self._publish_feedback(
                goal_handle,
                "dry_run",
                0.0,
                0.0,
                0.0,
                "dry run accepted",
            )
            return self._finish(
                goal_handle,
                True,
                0.0,
                "DRY_RUN: bin strafe accepted",
            )

        if not self._bool_parameter("enable_motion"):
            return self._finish(
                goal_handle,
                False,
                0.0,
                "PRECONDITION_FAILED: enable_motion is false",
            )

        started_at = time.monotonic()
        last_valid_at = started_at
        corner_count = 0
        period = 1.0 / max(1.0, self._double_parameter("control_rate_hz"))

        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - started_at
            if elapsed > timeout_sec:
                return self._finish(
                    goal_handle,
                    False,
                    elapsed,
                    "TIMEOUT: bin corner not reached",
                )
            if goal_handle.is_cancel_requested:
                self._publish_zero_twist()
                goal_handle.canceled()
                return self._result(False, elapsed, "cancelled")

            alignment, age = self._latest_alignment()
            alignment_is_fresh = (
                age <= self._double_parameter("alignment_timeout_sec")
            )
            if (
                self._alignment_matches_goal(
                    alignment,
                    goal.bed_id,
                    goal.side,
                )
                and alignment_is_fresh
            ):
                last_valid_at = now
                if alignment.corner_detected:
                    corner_count += 1
                else:
                    corner_count = 0

                if corner_count >= self._int_parameter("corner_confirmations"):
                    self._publish_zero_twist()
                    return self._finish(
                        goal_handle,
                        True,
                        elapsed,
                        "succeeded strafing: corner reached",
                    )

                self._publish_control_twist(
                    alignment,
                    direction,
                    target_distance,
                    strafe_speed,
                )
                self._publish_feedback(
                    goal_handle,
                    "strafing",
                    alignment.distance_error_m,
                    alignment.yaw_error_rad,
                    alignment.confidence,
                    alignment.message,
                )
            else:
                self._publish_zero_twist()
                self._publish_feedback(
                    goal_handle,
                    "lost_wall",
                    0.0,
                    0.0,
                    0.0,
                    "waiting for wall",
                )
                if now - last_valid_at > self._double_parameter(
                    "lost_wall_timeout_sec"
                ):
                    return self._finish(
                        goal_handle,
                        False,
                        elapsed,
                        "FAILED: wall alignment lost",
                    )

            time.sleep(period)

        return self._finish(
            goal_handle,
            False,
            time.monotonic() - started_at,
            "ROS shutdown",
        )

    def _latest_alignment(self) -> tuple[BinWallAlignment | None, float]:
        with self._lock:
            alignment = self._latest_alignment_msg
            age = time.monotonic() - self._latest_alignment_time
        return alignment, age

    def _alignment_matches_goal(
        self,
        alignment: BinWallAlignment | None,
        bed_id: str,
        side: str,
    ) -> bool:
        if alignment is None or not alignment.valid:
            return False
        if alignment.confidence < self._double_parameter("min_confidence"):
            return False
        if (
            bed_id.strip()
            and alignment.bed_id
            and alignment.bed_id != bed_id.strip()
        ):
            return False
        if (
            side.strip()
            and alignment.side
            and alignment.side.lower() != side.strip().lower()
        ):
            return False
        return True

    def _publish_control_twist(
        self,
        alignment: BinWallAlignment,
        direction: str,
        target_distance_m: float,
        strafe_speed_mps: float,
    ) -> None:
        distance_error = float(alignment.distance_m - target_distance_m)
        yaw_error = float(alignment.yaw_error_rad)

        twist = Twist()
        strafe_allowed = (
            math.isfinite(distance_error)
            and math.isfinite(yaw_error)
            and abs(distance_error) <= self._double_parameter(
                "strafe_distance_tolerance_m"
            )
            and abs(yaw_error) <= self._double_parameter(
                "strafe_yaw_tolerance_rad"
            )
        )
        if strafe_allowed:
            strafe_speed = min(
                strafe_speed_mps,
                self._double_parameter("max_strafe_speed_mps"),
            )
            twist.linear.y = self._strafe_sign(direction) * strafe_speed

        if (
            math.isfinite(distance_error)
            and abs(distance_error) > self._double_parameter(
                "distance_tolerance_m"
            )
        ):
            twist.linear.x = self._clamp(
                self._double_parameter("distance_gain") * distance_error,
                -self._double_parameter("max_distance_speed_mps"),
                self._double_parameter("max_distance_speed_mps"),
            )
        if (
            math.isfinite(yaw_error)
            and abs(yaw_error) > self._double_parameter(
                "yaw_tolerance_rad"
            )
        ):
            twist.angular.z = self._clamp(
                self._double_parameter("yaw_gain") * yaw_error,
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
        distance_error_m: float,
        yaw_error_rad: float,
        confidence: float,
        message: str,
    ) -> None:
        feedback = ExecuteBinStrafe.Feedback()
        feedback.phase = phase
        feedback.distance_error_m = float(distance_error_m)
        feedback.yaw_error_rad = float(yaw_error_rad)
        feedback.confidence = float(confidence)
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    def _finish(
        self,
        goal_handle,
        success: bool,
        elapsed_sec: float,
        message: str,
    ):
        self._publish_zero_twist()
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(message)
        return self._result(success, elapsed_sec, message)

    def _result(self, success: bool, elapsed_sec: float, message: str):
        result = ExecuteBinStrafe.Result()
        result.success = bool(success)
        result.elapsed_sec = float(elapsed_sec)
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
    def _strafe_sign(direction: str) -> float:
        return -1.0 if direction == "right" else 1.0

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BinStrafeActionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_zero_twist()
        except Exception:
            pass
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

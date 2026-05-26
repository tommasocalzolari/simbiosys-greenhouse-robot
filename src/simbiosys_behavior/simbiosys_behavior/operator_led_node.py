import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from mirte_msgs.srv import SetNeopixelSingle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from simbiosys_interfaces.msg import TaskStatus


@dataclass(frozen=True)
class LedColor:
    r: int
    g: int
    b: int

    def scaled(self, brightness: float) -> "LedColor":
        brightness = max(0.0, min(1.0, brightness))
        return LedColor(
            int(max(0, min(255, round(self.r * brightness)))),
            int(max(0, min(255, round(self.g * brightness)))),
            int(max(0, min(255, round(self.b * brightness)))),
        )


RED = LedColor(255, 0, 0)
GREEN = LedColor(0, 255, 0)
BLUE = LedColor(0, 0, 255)
ORANGE = LedColor(255, 120, 0)
OFF = LedColor(0, 0, 0)


class OperatorLedNode(Node):
    """Operator-facing LED strip feedback for robot mode and base motion."""

    def __init__(self) -> None:
        super().__init__("operator_led_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("led_count", 4)
        self.declare_parameter("one_based_led_numbers", False)
        self.declare_parameter(
            "set_single_service_name",
            "/io/leds/leds/set_color_single",
        )
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("task_status_topic", "simbiosys/task_status")
        self.declare_parameter("brightness", 0.35)
        self.declare_parameter("update_period_sec", 0.1)
        self.declare_parameter("blink_period_sec", 0.5)
        self.declare_parameter("turn_angular_threshold", 0.15)
        self.declare_parameter("strafe_linear_y_threshold", 0.03)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("reverse_led_order", False)
        self.declare_parameter("invert_turn_direction", True)
        self.declare_parameter("invert_strafe_direction", False)
        self.declare_parameter("missing_service_log_period_sec", 10.0)
        self.declare_parameter("left_turn_leds", [0, 1])
        self.declare_parameter("right_turn_leds", [2, 3])
        self.declare_parameter("left_strafe_leds", [3])
        self.declare_parameter("right_strafe_leds", [0])

        self._client = self.create_client(
            SetNeopixelSingle,
            self._string_parameter("set_single_service_name"),
        )

        self._latest_cmd_vel: Twist | None = None
        self._latest_cmd_vel_time = 0.0
        self._latest_task_status: TaskStatus | None = None
        self._latest_task_status_time = 0.0
        self._last_sent_colors: list[LedColor | None] = [
            None for _ in range(self._int_parameter("led_count"))
        ]
        self._pending_futures = []
        self._last_missing_service_log_time = 0.0

        self.create_subscription(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            self._on_cmd_vel,
            10,
        )
        self.create_subscription(
            TaskStatus,
            self._string_parameter("task_status_topic"),
            self._on_task_status,
            10,
        )

        period = max(0.05, self._double_parameter("update_period_sec"))
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "Operator LED node ready; targeting "
            f"{self._string_parameter('set_single_service_name')}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest_cmd_vel = msg
        self._latest_cmd_vel_time = time.monotonic()

    def _on_task_status(self, msg: TaskStatus) -> None:
        self._latest_task_status = msg
        self._latest_task_status_time = time.monotonic()

    def _on_timer(self) -> None:
        self._pending_futures = [
            future for future in self._pending_futures if not future.done()
        ]

        desired_colors = self._desired_colors()
        if not self._bool_parameter("enabled"):
            desired_colors = [OFF for _ in desired_colors]

        self._publish_changed_colors(desired_colors)

    def _desired_colors(self) -> list[LedColor]:
        led_count = self._int_parameter("led_count")
        base_color = self._base_mode_color()
        colors = [base_color for _ in range(led_count)]

        cmd_vel = self._fresh_cmd_vel()
        if cmd_vel is None:
            return colors

        blink_on = self._blink_on()
        turn_threshold = self._double_parameter("turn_angular_threshold")
        strafe_threshold = self._double_parameter("strafe_linear_y_threshold")
        angular_z = cmd_vel.angular.z
        linear_y = cmd_vel.linear.y

        if self._bool_parameter("invert_turn_direction"):
            angular_z *= -1.0
        if self._bool_parameter("invert_strafe_direction"):
            linear_y *= -1.0

        if angular_z > turn_threshold:
            self._apply_blink(colors, self._led_indices("left_turn_leds"), blink_on)
        elif angular_z < -turn_threshold:
            self._apply_blink(colors, self._led_indices("right_turn_leds"), blink_on)

        if linear_y > strafe_threshold:
            self._apply_blink(colors, self._led_indices("left_strafe_leds"), blink_on)
        elif linear_y < -strafe_threshold:
            self._apply_blink(colors, self._led_indices("right_strafe_leds"), blink_on)

        return colors

    def _base_mode_color(self) -> LedColor:
        status = self._latest_task_status
        if status is None:
            return GREEN

        if status.error:
            return RED

        state = status.current_state.strip().upper()
        message = status.message.strip().upper()
        if "TELEOP" in state or "TELEOP" in message:
            return BLUE

        return GREEN

    def _fresh_cmd_vel(self) -> Twist | None:
        if self._latest_cmd_vel is None:
            return None

        timeout = max(0.05, self._double_parameter("cmd_vel_timeout_sec"))
        if time.monotonic() - self._latest_cmd_vel_time > timeout:
            return None

        return self._latest_cmd_vel

    def _blink_on(self) -> bool:
        period = max(0.1, self._double_parameter("blink_period_sec"))
        return int(time.monotonic() / period) % 2 == 0

    def _apply_blink(
        self,
        colors: list[LedColor],
        indices: list[int],
        blink_on: bool,
    ) -> None:
        for index in indices:
            if 0 <= index < len(colors):
                colors[index] = ORANGE if blink_on else OFF

    def _publish_changed_colors(self, desired_colors: list[LedColor]) -> None:
        if not self._client.service_is_ready():
            self._log_missing_service()
            self._last_sent_colors = [None for _ in desired_colors]
            return

        brightness = self._double_parameter("brightness")
        for index, color in enumerate(desired_colors):
            scaled_color = color.scaled(brightness)
            if index < len(self._last_sent_colors) and self._last_sent_colors[index] == scaled_color:
                continue

            request = SetNeopixelSingle.Request()
            request.led_index = index
            request.color.r = scaled_color.r
            request.color.g = scaled_color.g
            request.color.b = scaled_color.b

            future = self._client.call_async(request)
            future.add_done_callback(self._on_led_response)
            self._pending_futures.append(future)
            self._last_sent_colors[index] = scaled_color

    def _on_led_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive ROS callback logging
            self.get_logger().warning(f"LED service call failed: {exc}")
            return

        if not response.status:
            self.get_logger().warning("LED service rejected a requested LED color")

    def _log_missing_service(self) -> None:
        now = time.monotonic()
        period = max(1.0, self._double_parameter("missing_service_log_period_sec"))
        if now - self._last_missing_service_log_time < period:
            return

        self._last_missing_service_log_time = now
        self.get_logger().warning(
            "LED service is not available yet: "
            f"{self._string_parameter('set_single_service_name')}"
        )

    def _led_indices(self, parameter_name: str) -> list[int]:
        raw_indices = self.get_parameter(parameter_name).value
        offset = 1 if self._bool_parameter("one_based_led_numbers") else 0
        indices = [int(index) - offset for index in raw_indices]

        if self._bool_parameter("reverse_led_order"):
            led_count = self._int_parameter("led_count")
            indices = [led_count - 1 - index for index in indices]

        return indices

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _double_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorLedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

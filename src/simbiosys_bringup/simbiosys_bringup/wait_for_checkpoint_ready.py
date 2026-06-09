import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaitForCheckpointReady(Node):
    """Exit after checkpoint navigation reports a ready route."""

    def __init__(self) -> None:
        super().__init__("wait_for_checkpoint_ready")
        self.declare_parameter("status_topic", "/checkpoint_status")
        self.declare_parameter("timeout_sec", 0.0)
        self._ready = False
        self._started_at = time.monotonic()
        self.create_subscription(
            String,
            self.get_parameter("status_topic").get_parameter_value().string_value,
            self._on_status,
            10,
        )
        self.create_timer(0.2, self._on_timer)

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(status, dict):
            return
        if int(status.get("route_length", 0)) <= 0:
            return
        event = str(status.get("event", ""))
        state = str(status.get("state", ""))
        if event == "ready" or state == "waiting":
            self.get_logger().info("Checkpoint navigator is ready")
            self._ready = True

    def _on_timer(self) -> None:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        if timeout_sec <= 0.0 or self._ready:
            return
        if time.monotonic() - self._started_at >= timeout_sec:
            self.get_logger().warning(
                "Still waiting for checkpoint navigator readiness"
            )
            self._started_at = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaitForCheckpointReady()
    try:
        while rclpy.ok() and not node._ready:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

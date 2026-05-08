import rclpy
from rclpy.node import Node

from simbiosys_interfaces.msg import FlowerData, TaskStatus
from simbiosys_interfaces.srv import SetRobotMode


class UiNode(Node):
    """Terminal dashboard for first-period SimBioSys development."""

    def __init__(self) -> None:
        super().__init__("ui_node")
        self._set_mode_client = self.create_client(
            SetRobotMode,
            "simbiosys/set_robot_mode",
        )
        self.create_subscription(
            TaskStatus,
            "simbiosys/task_status",
            self._on_task_status,
            10,
        )
        self.create_subscription(
            FlowerData,
            "simbiosys/flower_data",
            self._on_flower_data,
            10,
        )
        self._timer = self.create_timer(5.0, self._on_timer)

        # TODO: Replace terminal logging with a dashboard once the team agrees
        # on the operator workflow.
        # TODO: Call SetRobotMode when the operator changes mission mode.
        self.get_logger().info("Terminal UI started")

    def _on_timer(self) -> None:
        if not self._set_mode_client.service_is_ready():
            self.get_logger().info("Waiting for simbiosys/set_robot_mode")

    def _on_task_status(self, msg: TaskStatus) -> None:
        self.get_logger().info(
            f"TaskStatus state={msg.current_state} active={msg.active} "
            f"error={msg.error} message='{msg.message}'"
        )

    def _on_flower_data(self, msg: FlowerData) -> None:
        self.get_logger().info(
            f"FlowerData detected={msg.detected} confidence={msg.confidence:.2f} "
            f"label='{msg.label}' message='{msg.message}'"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UiNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

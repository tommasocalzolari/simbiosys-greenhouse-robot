import rclpy
from rclpy.node import Node


class TeleopInterfaceNode(Node):
    """Teleoperation interface extension point."""

    def __init__(self) -> None:
        super().__init__("teleop_interface_node")
        self._timer = self.create_timer(2.0, self._on_timer)

        # TODO: Connect operator commands to safe high-level teleop hooks.
        # TODO: Avoid bypassing robot-side safety and low-level controllers.
        self.get_logger().info("Teleop interface extension point started")

    def _on_timer(self) -> None:
        self.get_logger().info("Waiting for teleop interface implementation")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopInterfaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

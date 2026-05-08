import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMonitorNode(Node):
    """Log MIRTE joint state names and positions at a human-readable rate."""

    def __init__(self) -> None:
        super().__init__("joint_state_monitor_node")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("log_period_sec", 2.0)

        self._latest_msg = None
        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        log_period_sec = (
            self.get_parameter("log_period_sec").get_parameter_value().double_value
        )

        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_state,
            10,
        )
        self.create_timer(log_period_sec, self._on_timer)

        self.get_logger().info(
            f"Monitoring joint states from {self._joint_states_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_msg = msg

    def _on_timer(self) -> None:
        if self._latest_msg is None:
            self.get_logger().info(
                f"Waiting for JointState messages on {self._joint_states_topic}"
            )
            return

        pairs = []
        for name, position in zip(self._latest_msg.name, self._latest_msg.position):
            pairs.append(f"{name}={position:.3f}")
        self.get_logger().info("Joint positions: " + ", ".join(pairs))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

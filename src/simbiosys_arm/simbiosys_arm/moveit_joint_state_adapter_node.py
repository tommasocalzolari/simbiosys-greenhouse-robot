import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAME_REMAP = {
    "_Gripper_joint_r_mimic": "_Gripper_joint_r",
    "_gripper_link_joint_l_mimic": "_gripper_link_joint_l",
    "_gripper_link_joint_r_mimic": "_gripper_link_joint_r",
}


class MoveItJointStateAdapterNode(Node):
    """Republish sim joint states with names matching the MoveIt robot model."""

    def __init__(self) -> None:
        super().__init__("moveit_joint_state_adapter_node")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/simbiosys/moveit_joint_states")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_joint_state, 10)
        self.get_logger().info(
            f"Republishing MoveIt-compatible joint states: {input_topic} -> {output_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        fixed = JointState()
        fixed.header = msg.header

        for index, name in enumerate(msg.name):
            fixed.name.append(JOINT_NAME_REMAP.get(name, name))
            if index < len(msg.position):
                fixed.position.append(msg.position[index])
            if index < len(msg.velocity):
                fixed.velocity.append(msg.velocity[index])
            if index < len(msg.effort):
                fixed.effort.append(msg.effort[index])

        self._publisher.publish(fixed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItJointStateAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

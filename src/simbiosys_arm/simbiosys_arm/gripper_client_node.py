import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool


class GripperClientNode(Node):
    """Wrapper around the MIRTE gripper action with open/close placeholders."""

    def __init__(self) -> None:
        super().__init__("gripper_client_node")
        self.declare_parameter(
            "gripper_action",
            "/mirte_master_gripper_controller/gripper_cmd",
        )
        self.declare_parameter("open_position", 0.04)
        self.declare_parameter("close_position", 0.0)
        self.declare_parameter("max_effort", 5.0)

        self._action_name = (
            self.get_parameter("gripper_action").get_parameter_value().string_value
        )
        self._open_position = (
            self.get_parameter("open_position").get_parameter_value().double_value
        )
        self._close_position = (
            self.get_parameter("close_position").get_parameter_value().double_value
        )
        self._max_effort = (
            self.get_parameter("max_effort").get_parameter_value().double_value
        )

        self._client = ActionClient(self, GripperCommand, self._action_name)
        self.create_service(SetBool, "simbiosys/set_gripper_closed", self._on_set_closed)

        self.get_logger().info(
            "Gripper client ready. Call simbiosys/set_gripper_closed with "
            "data=true to close or data=false to open."
        )

    def _on_set_closed(self, request, response):
        position = self._close_position if request.data else self._open_position
        label = "close" if request.data else "open"

        if not self._client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = f"Gripper action {self._action_name} is not available"
            self.get_logger().warn(response.message)
            return response

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self._max_effort
        self._client.send_goal_async(goal)

        response.success = True
        response.message = f"Sent placeholder gripper {label} goal"
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperClientNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

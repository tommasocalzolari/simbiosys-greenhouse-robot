import rclpy
from rclpy.node import Node

from simbiosys_behavior.state_machine import MissionStateMachine
from simbiosys_interfaces.msg import TaskStatus
from simbiosys_interfaces.srv import SetRobotMode


class MissionManagerNode(Node):
    """Laptop-side coordinator for the SimBioSys mission."""

    def __init__(self) -> None:
        super().__init__("mission_manager_node")
        self._state_machine = MissionStateMachine()
        self._status_publisher = self.create_publisher(
            TaskStatus,
            "simbiosys/task_status",
            10,
        )
        self.create_service(
            SetRobotMode,
            "simbiosys/set_robot_mode",
            self._on_set_robot_mode,
        )
        self._timer = self.create_timer(1.0, self._on_timer)

        # This node deliberately manages modes only for now. Existing MIRTE,
        # slam_toolbox, MoveIt, and ros2_control nodes own the low-level work.
        self.get_logger().info("Mission manager started in reuse-first mode")

    def _on_timer(self) -> None:
        state = self._state_machine.step()

        status = TaskStatus()
        status.current_state = state.value
        status.active = state.value != "ERROR"
        status.error = state.value == "ERROR"
        status.message = "Placeholder mission manager is running"

        self._status_publisher.publish(status)
        self.get_logger().info(f"Current mission state: {state.value}")

    def _on_set_robot_mode(self, request, response):
        success, message = self._state_machine.set_mode(request.mode)
        response.success = success
        response.message = message
        self.get_logger().info(message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

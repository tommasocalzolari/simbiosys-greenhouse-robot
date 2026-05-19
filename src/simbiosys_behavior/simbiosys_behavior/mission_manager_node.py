import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from simbiosys_behavior.state_machine import MissionStateMachine
from simbiosys_interfaces.action import ExecuteBehavior
from simbiosys_interfaces.msg import BehaviorType, TaskStatus
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
        self._execute_behavior_server = ActionServer(
            self,
            ExecuteBehavior,
            "simbiosys/execute_behavior",
            self._on_execute_behavior,
        )
        self._timer = self.create_timer(1.0, self._on_timer)

        # This node deliberately manages modes only for now. Existing MIRTE,
        # slam_toolbox, MoveIt, and ros2_control nodes own the low-level work.
        self.get_logger().info(
            "Mission manager started with SetRobotMode and ExecuteBehavior"
        )

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

    def _on_execute_behavior(self, goal_handle):
        behavior = goal_handle.request.behavior.type
        target_id = goal_handle.request.target_id
        behavior_name = self._behavior_name(behavior)

        feedback = ExecuteBehavior.Feedback()
        feedback.current_step = f"Accepted {behavior_name}"
        feedback.progress = 0.25
        goal_handle.publish_feedback(feedback)

        success, message = self._set_mode_for_behavior(behavior)

        feedback.current_step = message
        feedback.progress = 1.0 if success else 0.0
        goal_handle.publish_feedback(feedback)

        result = ExecuteBehavior.Result()
        result.success = success
        result.message = (
            f"{behavior_name} requested for {target_id or 'no target'}: {message}"
        )

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(result.message)
        return result

    def _set_mode_for_behavior(self, behavior: int) -> tuple[bool, str]:
        mode_by_behavior = {
            BehaviorType.IDLE: "AUTONOMOUS_IDLE",
            BehaviorType.TELEOP: "TELEOP",
            BehaviorType.MAP: "MAPPING",
            BehaviorType.LOCALIZE: "MAPPING",
            BehaviorType.INSPECT_BED: "AUTONOMOUS_IDLE",
            BehaviorType.INSPECT_FLOWER: "AUTONOMOUS_IDLE",
            BehaviorType.HARVEST: "AUTONOMOUS_IDLE",
            BehaviorType.ARM_TEST: "ARM_TEST",
        }
        mode = mode_by_behavior.get(behavior)
        if mode is None:
            return False, f"Unknown behavior type {behavior}"
        return self._state_machine.set_mode(mode)

    def _behavior_name(self, behavior: int) -> str:
        names = {
            BehaviorType.IDLE: "IDLE",
            BehaviorType.TELEOP: "TELEOP",
            BehaviorType.MAP: "MAP",
            BehaviorType.LOCALIZE: "LOCALIZE",
            BehaviorType.INSPECT_BED: "INSPECT_BED",
            BehaviorType.INSPECT_FLOWER: "INSPECT_FLOWER",
            BehaviorType.HARVEST: "HARVEST",
            BehaviorType.ARM_TEST: "ARM_TEST",
        }
        return names.get(behavior, f"UNKNOWN_{behavior}")


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

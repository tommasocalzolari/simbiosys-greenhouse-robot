import copy
import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from simbiosys_interfaces.action import AnalyzePlantScan, ExecuteBehavior
from simbiosys_interfaces.msg import (
    BehaviorType,
    CurrentMission,
    ScanPosition,
    TaskStatus,
)


class DemoCheckpointMissionNode(Node):
    """Run checkpoint navigation and one-image perception in a small loop."""

    def __init__(self) -> None:
        super().__init__("demo_checkpoint_mission_node")
        self._callback_group = ReentrantCallbackGroup()
        self._status_condition = threading.Condition()
        self._latest_checkpoint_status = None
        self._checkpoint_status_seq = 0
        self._mission_active = False
        self._stop_requested = threading.Event()
        self._active_analysis_goal = None
        self._latest_plant_health = None

        self.declare_parameter(
            "checkpoint_command_topic",
            "/checkpoint_commands",
        )
        self.declare_parameter("checkpoint_status_topic", "/checkpoint_status")
        self.declare_parameter(
            "plant_analysis_action_name",
            "simbiosys/analyze_plant_scan",
        )
        self.declare_parameter("checkpoint_timeout_sec", 180.0)
        self.declare_parameter("analysis_server_timeout_sec", 10.0)
        self.declare_parameter("analysis_timeout_sec", 10.0)
        self.declare_parameter("analysis_dry_run", False)
        self.declare_parameter("checkpoints_per_bed", 4)
        self.declare_parameter("checkpoints_per_side", 2)

        self._command_pub = self.create_publisher(
            String,
            self._string_parameter("checkpoint_command_topic"),
            10,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            self._string_parameter("checkpoint_status_topic"),
            self._on_checkpoint_status,
            status_qos,
            callback_group=self._callback_group,
        )
        self._task_status_pub = self.create_publisher(
            TaskStatus,
            "simbiosys/task_status",
            10,
        )
        self._current_mission_pub = self.create_publisher(
            CurrentMission,
            "simbiosys/current_mission",
            10,
        )
        self._analysis_client = ActionClient(
            self,
            AnalyzePlantScan,
            self._string_parameter("plant_analysis_action_name"),
            callback_group=self._callback_group,
        )
        self._behavior_server = ActionServer(
            self,
            ExecuteBehavior,
            "simbiosys/execute_behavior",
            execute_callback=self._execute_behavior,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._publish_task_status(
            "WAIT_FOR_START",
            "Waiting for UI start",
            False,
        )
        self.get_logger().info(
            "Demo mission ready: checkpoint -> fresh image analysis "
            "-> next checkpoint"
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _goal_callback(self, request: ExecuteBehavior.Goal) -> GoalResponse:
        behavior = request.behavior.type
        if behavior == BehaviorType.IDLE:
            return GoalResponse.ACCEPT
        if behavior != BehaviorType.INSPECT_BED:
            return GoalResponse.REJECT
        with self._status_condition:
            if self._mission_active:
                return GoalResponse.REJECT
            self._mission_active = True
            self._stop_requested.clear()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._request_stop()
        return CancelResponse.ACCEPT

    def _execute_behavior(self, goal_handle):
        if goal_handle.request.behavior.type == BehaviorType.IDLE:
            self._request_stop()
            self._publish_command("cancel")
            self._publish_task_status(
                "WAIT_FOR_START",
                "Mission stopped",
                False,
            )
            return self._finish(goal_handle, True, "Mission stopped")

        mission_id = f"demo-{int(time.time())}"
        try:
            return self._run_mission(goal_handle, mission_id)
        finally:
            with self._status_condition:
                self._mission_active = False
                self._status_condition.notify_all()

    def _run_mission(self, goal_handle, mission_id: str):
        self._publish_task_status(
            "RESET_ROUTE",
            "Resetting checkpoint route",
            True,
        )
        start_seq = self._checkpoint_snapshot()[1]
        self._publish_command("reset")
        ready = self._wait_for_checkpoint_event(
            goal_handle,
            start_seq,
            {"reset", "ready", "reloaded"},
            10.0,
        )
        if not isinstance(ready, dict):
            return self._fail(goal_handle, mission_id, str(ready))

        route_length = int(ready.get("route_length", 0))
        if route_length <= 0:
            return self._fail(
                goal_handle,
                mission_id,
                "Checkpoint route is empty",
            )

        for index in range(route_length):
            next_target = ready.get("next_target")
            label = self._target_label(next_target, index)
            self._publish_mission(
                mission_id,
                "navigating",
                f"Navigating to {label}",
                index,
                route_length,
                next_target,
            )
            self._publish_task_status(
                "NAVIGATE_TO_CHECKPOINT",
                f"Navigating to {label}",
                True,
            )

            start_seq = self._checkpoint_snapshot()[1]
            self._publish_command("next")
            arrived = self._wait_for_checkpoint_event(
                goal_handle,
                start_seq,
                {"arrived"},
                self._double_parameter("checkpoint_timeout_sec"),
            )
            if not isinstance(arrived, dict):
                return self._fail(goal_handle, mission_id, str(arrived))

            arrived_target = arrived.get("arrived_target")
            if self._is_terminal_target(arrived_target):
                label = self._target_label(arrived_target, index)
                message = (
                    f"Demo complete: analyzed {index} checkpoints and reached {label}"
                )
                self._publish_mission(
                    mission_id,
                    "complete",
                    message,
                    index + 1,
                    route_length,
                    arrived_target,
                )
                self._publish_task_status("COMPLETE", message, False)
                return self._finish(goal_handle, True, message)

            scan_position, side, lane = self._scan_target(arrived_target)
            if scan_position is None:
                return self._fail(
                    goal_handle,
                    mission_id,
                    "Arrived checkpoint has no scan metadata",
                )

            label = self._target_label(arrived_target, index)
            self._publish_mission(
                mission_id,
                "analyzing",
                f"Taking and analyzing one image at {label}",
                index,
                route_length,
                arrived_target,
            )
            self._publish_task_status(
                "ANALYZE_ONE_IMAGE",
                f"Analyzing {label}",
                True,
            )
            success, message = self._analyze(
                goal_handle,
                mission_id,
                scan_position,
                side,
                lane,
            )
            if not success:
                return self._fail(goal_handle, mission_id, message)

            ready = arrived
            self._publish_mission(
                mission_id,
                "checkpoint_complete",
                f"Completed {label}: {message}",
                index + 1,
                route_length,
                arrived_target,
            )

        message = f"Demo complete: analyzed {route_length} checkpoints"
        self._publish_mission(
            mission_id,
            "complete",
            message,
            route_length,
            route_length,
        )
        self._publish_task_status("COMPLETE", message, False)
        return self._finish(goal_handle, True, message)

    def _on_checkpoint_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("Ignoring malformed checkpoint status")
            return
        with self._status_condition:
            self._latest_checkpoint_status = status
            self._checkpoint_status_seq += 1
            self._status_condition.notify_all()

    def _checkpoint_snapshot(self):
        with self._status_condition:
            return (
                copy.deepcopy(self._latest_checkpoint_status),
                self._checkpoint_status_seq,
            )

    def _wait_for_checkpoint_event(
        self,
        goal_handle,
        start_seq: int,
        accepted_events: set[str],
        timeout_sec: float,
    ):
        deadline = time.monotonic() + timeout_sec
        with self._status_condition:
            while rclpy.ok():
                if (
                    goal_handle.is_cancel_requested
                    or self._stop_requested.is_set()
                ):
                    return "Mission stopped"
                status = self._latest_checkpoint_status
                if (
                    status is not None
                    and self._checkpoint_status_seq > start_seq
                ):
                    event = str(status.get("event", ""))
                    if status.get("error") or event in {
                        "failed",
                        "nav2_unavailable",
                        "rejected",
                    }:
                        return str(
                            status.get(
                                "message",
                                "Checkpoint navigation failed",
                            )
                        )
                    if event in accepted_events:
                        return copy.deepcopy(status)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return "Timed out waiting for checkpoint navigator"
                self._status_condition.wait(timeout=min(0.1, remaining))
        return "ROS shutdown"

    def _analyze(
        self,
        goal_handle,
        mission_id: str,
        scan_position: ScanPosition,
        side: str,
        lane: str,
    ) -> tuple[bool, str]:
        if not self._analysis_client.wait_for_server(
            timeout_sec=self._double_parameter("analysis_server_timeout_sec")
        ):
            return False, "Plant analysis action is unavailable"

        goal = AnalyzePlantScan.Goal()
        goal.scan_position = scan_position
        goal.side = side
        goal.mission_id = mission_id
        goal.request_id = (
            f"{mission_id}:{scan_position.scan_position_id}:{side}:lane={lane}"
        )
        goal.timeout_sec = self._double_parameter("analysis_timeout_sec")
        goal.dry_run = self._bool_parameter("analysis_dry_run")

        completed, analysis_goal = self._wait_for_future(
            goal_handle,
            self._analysis_client.send_goal_async(goal),
            5.0,
        )
        if (
            not completed
            or analysis_goal is None
            or not analysis_goal.accepted
        ):
            return False, str(
                analysis_goal or "Plant analysis goal was rejected"
            )

        self._active_analysis_goal = analysis_goal
        completed, wrapped_result = self._wait_for_future(
            goal_handle,
            analysis_goal.get_result_async(),
            self._double_parameter("analysis_timeout_sec") + 5.0,
        )
        self._active_analysis_goal = None
        if not completed or wrapped_result is None:
            return False, str(
                wrapped_result or "Plant analysis returned no result"
            )
        if int(wrapped_result.status) != GoalStatus.STATUS_SUCCEEDED:
            return (
                False,
                "Plant analysis finished with status "
                f"{wrapped_result.status}",
            )

        result = wrapped_result.result
        if result.success:
            self._latest_plant_health = result.plant_health
        message = result.message or "Plant analysis complete"
        return bool(result.success), message

    def _wait_for_future(self, goal_handle, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if (
                goal_handle.is_cancel_requested
                or self._stop_requested.is_set()
            ):
                return False, "Mission stopped"
            if time.monotonic() >= deadline:
                return False, "Action timed out"
            time.sleep(0.05)
        if not future.done():
            return False, "ROS shutdown"
        try:
            return True, future.result()
        except Exception as exc:
            return False, f"Action failed: {exc}"

    def _scan_target(self, target):
        if not isinstance(target, dict):
            return None, "", ""
        metadata = target.get("metadata")
        if not isinstance(metadata, dict):
            return None, "", ""

        order = self._target_order(target, metadata)
        bed_id = str(metadata.get("bed_id", "")).strip()
        side = str(metadata.get("side", "")).strip().lower()
        lane = str(metadata.get("lane", "")).strip().lower()
        scan_id = str(metadata.get("scan_position_id", "")).strip()
        if order > 0 and (not bed_id or not side or not scan_id):
            derived = self._derived_scan_metadata(order)
            bed_id = bed_id or derived[0]
            side = side or derived[1]
            scan_id = scan_id or derived[2]
        if not scan_id:
            scan_id = str(target.get("label", "")).strip()
        if not scan_id:
            return None, side, lane

        scan_position = ScanPosition()
        scan_position.scan_position_id = scan_id
        scan_position.bed_id = bed_id
        scan_position.order = max(0, order)
        scan_position.enabled = True
        pose = target.get("pose")
        if isinstance(pose, dict):
            position = pose.get("position", {})
            scan_position.base_pose.x = float(position.get("x", 0.0))
            scan_position.base_pose.y = float(position.get("y", 0.0))
        return scan_position, side, lane

    @staticmethod
    def _is_terminal_target(target) -> bool:
        if not isinstance(target, dict):
            return False
        metadata = target.get("metadata")
        if not isinstance(metadata, dict):
            return False
        return bool(metadata.get("terminal", False)) and not bool(
            metadata.get("run_perception", True)
        )

    @staticmethod
    def _target_order(target: dict, metadata: dict) -> int:
        try:
            order = int(metadata.get("order", 0))
        except (TypeError, ValueError):
            order = 0
        if order > 0:
            return order
        label = str(target.get("label", "")).strip()
        try:
            return int(label.rsplit("_", 1)[1])
        except (IndexError, ValueError):
            return 0

    def _derived_scan_metadata(self, order: int) -> tuple[str, str, str]:
        checkpoints_per_bed = max(
            1,
            self._int_parameter("checkpoints_per_bed"),
        )
        checkpoints_per_side = max(
            1,
            self._int_parameter("checkpoints_per_side"),
        )
        index_in_bed = (order - 1) % checkpoints_per_bed
        bed_id = str(((order - 1) // checkpoints_per_bed) + 1)
        side_index = index_in_bed // checkpoints_per_side
        side = chr(ord("a") + side_index)
        position_in_side = (index_in_bed % checkpoints_per_side) + 1
        scan_id = f"bed_{bed_id}_{side}_{position_in_side}"
        return bed_id, side, scan_id

    @staticmethod
    def _target_label(target, index: int) -> str:
        if isinstance(target, dict) and str(target.get("label", "")).strip():
            return str(target["label"])
        return f"checkpoint_{index + 1}"

    def _publish_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._command_pub.publish(msg)

    def _publish_task_status(
        self,
        state: str,
        message: str,
        active: bool,
    ) -> None:
        msg = TaskStatus()
        msg.current_state = state
        msg.active = active
        msg.error = state == "ERROR"
        msg.message = message
        self._task_status_pub.publish(msg)

    def _publish_mission(
        self,
        mission_id: str,
        phase: str,
        message: str,
        queue_index: int,
        queue_total: int,
        target=None,
        error: bool = False,
    ) -> None:
        msg = CurrentMission()
        msg.mission_id = mission_id
        msg.phase = phase
        msg.queue_index = max(0, queue_index)
        msg.queue_total = max(0, queue_total)
        msg.error = error
        msg.message = message
        if self._latest_plant_health is not None:
            msg.latest_plant_health = self._latest_plant_health
        if isinstance(target, dict):
            metadata = target.get("metadata", {})
            msg.active_bed_id = str(metadata.get("bed_id", ""))
            msg.active_side = str(metadata.get("side", ""))
            msg.active_scan_position_id = str(
                metadata.get("scan_position_id", "")
            )
            msg.target_pose = self._pose_from_target(target)
        self._current_mission_pub.publish(msg)

    def _pose_from_target(self, target) -> PoseStamped:
        msg = PoseStamped()
        pose = target.get("pose", {})
        msg.header.frame_id = str(pose.get("frame_id", "map"))
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        msg.pose.position.x = float(position.get("x", 0.0))
        msg.pose.position.y = float(position.get("y", 0.0))
        msg.pose.position.z = float(position.get("z", 0.0))
        msg.pose.orientation.x = float(orientation.get("x", 0.0))
        msg.pose.orientation.y = float(orientation.get("y", 0.0))
        msg.pose.orientation.z = float(orientation.get("z", 0.0))
        msg.pose.orientation.w = float(orientation.get("w", 1.0))
        return msg

    def _request_stop(self) -> None:
        self._stop_requested.set()
        with self._status_condition:
            self._status_condition.notify_all()
        if self._active_analysis_goal is not None:
            self._active_analysis_goal.cancel_goal_async()

    def _fail(self, goal_handle, mission_id: str, message: str):
        self._publish_command("cancel")
        self._publish_mission(mission_id, "failed", message, 0, 0, error=True)
        self._publish_task_status("ERROR", message, False)
        return self._finish(goal_handle, False, message)

    @staticmethod
    def _finish(goal_handle, success: bool, message: str):
        result = ExecuteBehavior.Result()
        result.success = success
        result.message = message
        if goal_handle.is_cancel_requested:
            result.success = False
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoCheckpointMissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._request_stop()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

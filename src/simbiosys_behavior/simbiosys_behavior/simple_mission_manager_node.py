import copy
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
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

from simbiosys_interfaces.action import (
    AnalyzePlantScan,
    ExecuteBehavior,
    ExecuteScanPosition,
)
from simbiosys_interfaces.msg import (
    BehaviorType,
    CurrentMission,
    PlantHealth,
    ScanPosition,
    ScanProgress,
    TaskStatus,
)


@dataclass
class MissionTarget:
    label: str
    scan_position: ScanPosition
    side: str
    pose: PoseStamped
    target_distance_m: float = 0.0


class SimpleMissionManagerNode(Node):
    """Drive each configured checkpoint, align, and analyze one fresh image."""

    def __init__(self) -> None:
        super().__init__("simple_mission_manager_node")
        self._callback_group = ReentrantCallbackGroup()
        self._mission_lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._mission_reserved = False
        self._active_nav_goal = None
        self._active_scan_goal = None
        self._active_analysis_goal = None
        self._latest_pose = PoseStamped()
        self._latest_plant_health = PlantHealth()
        self._state = "WAIT_FOR_OPERATOR"
        self._state_message = "Waiting for operator"
        self._initial_pose_remaining = 0

        self.declare_parameter(
            "annotations_file",
            "maps/mirte_map_annotations.json",
        )
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter(
            "scan_position_action_name",
            "simbiosys/execute_scan_position",
        )
        self.declare_parameter(
            "plant_analysis_action_name",
            "simbiosys/analyze_plant_scan",
        )
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter(
            "cmd_vel_topic",
            "/mirte_base_controller/cmd_vel",
        )
        self.declare_parameter("action_server_timeout_sec", 10.0)
        self.declare_parameter("navigation_timeout_sec", 180.0)
        self.declare_parameter("scan_position_dry_run", True)
        self.declare_parameter("plant_analysis_dry_run", True)
        self.declare_parameter("scan_target_distance_m", 0.35)
        self.declare_parameter("scan_hold_duration_sec", 1.0)
        self.declare_parameter("plant_analysis_timeout_sec", 5.0)
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose_publish_count", 10)
        self.declare_parameter("initial_pose_covariance_xy", 0.25)
        self.declare_parameter("initial_pose_covariance_yaw", 0.0685)

        self._targets, self._home_pose = self._load_route(
            self._string_parameter("annotations_file")
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
        self._scan_progress_pub = self.create_publisher(
            ScanProgress,
            "simbiosys/scan_progress",
            10,
        )
        checkpoint_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._checkpoint_status_pub = self.create_publisher(
            String,
            "/checkpoint_status",
            checkpoint_qos,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self._string_parameter("initial_pose_topic"),
            10,
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._string_parameter("amcl_pose_topic"),
            self._on_amcl_pose,
            10,
        )

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self._string_parameter("nav2_action_name"),
            callback_group=self._callback_group,
        )
        self._scan_client = ActionClient(
            self,
            ExecuteScanPosition,
            self._string_parameter("scan_position_action_name"),
            callback_group=self._callback_group,
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

        if self._bool_parameter("publish_initial_pose"):
            self._initial_pose_remaining = self._int_parameter(
                "initial_pose_publish_count"
            )
        self.create_timer(1.0, self._on_timer)
        self._publish_checkpoint_status(
            "ready",
            "Simple mission route ready",
            0,
            self._targets[0],
        )
        self.get_logger().info(
            "Simple mission manager ready with "
            f"{len(self._targets)} checkpoints"
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
        if behavior in (BehaviorType.IDLE, BehaviorType.TELEOP):
            return GoalResponse.ACCEPT
        if behavior != BehaviorType.INSPECT_BED:
            self.get_logger().warning(
                "Only IDLE, TELEOP, and INSPECT_BED are supported"
            )
            return GoalResponse.REJECT
        with self._mission_lock:
            if self._mission_reserved:
                self.get_logger().warning(
                    "Another checkpoint mission is active"
                )
                return GoalResponse.REJECT
            self._stop_requested.clear()
            self._mission_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._request_stop("Mission cancellation requested")
        return CancelResponse.ACCEPT

    def _execute_behavior(self, goal_handle):
        behavior = goal_handle.request.behavior.type
        if behavior == BehaviorType.IDLE:
            self._request_stop("Operator requested IDLE")
            self._set_state("AUTONOMOUS_IDLE", "Robot idle")
            return self._finish(goal_handle, True, "Robot set to IDLE")
        if behavior == BehaviorType.TELEOP:
            self._request_stop("Operator requested TELEOP")
            self._set_state("TELEOP", "UI teleop owns control")
            return self._finish(goal_handle, True, "Robot set to TELEOP")

        try:
            return self._run_mission(goal_handle)
        finally:
            with self._mission_lock:
                self._mission_reserved = False
            self._publish_task_status()

    def _run_mission(self, goal_handle):
        mission_id = f"checkpoint-{int(time.time())}"
        target_filter = goal_handle.request.target_id.strip()
        targets = [
            target
            for target in self._targets
            if self._target_matches(target_filter, target)
        ]
        if not targets:
            return self._mission_failed(
                goal_handle,
                mission_id,
                f"No checkpoints match '{target_filter}'",
            )

        self._set_state("SCANNING", "Checkpoint mission active")
        self._publish_checkpoint_status("started", "Mission started", 0)

        for index, target in enumerate(targets):
            stop_message = self._stop_message(goal_handle)
            if stop_message is not None:
                return self._mission_stopped(
                    goal_handle,
                    mission_id,
                    stop_message,
                )

            self._publish_current_mission(
                mission_id,
                "navigating",
                f"Navigating to {target.label}",
                target,
                index,
                len(targets),
            )
            self._publish_checkpoint_status(
                "navigating",
                f"Navigating to {target.label}",
                index,
                target,
            )
            navigated, message = self._navigate(goal_handle, target)
            if not navigated:
                return self._mission_failed(
                    goal_handle,
                    mission_id,
                    message,
                    index,
                    len(targets),
                    target,
                )

            self._publish_current_mission(
                mission_id,
                "aligning",
                f"Aligning at {target.label}",
                target,
                index,
                len(targets),
            )
            aligned, message = self._align(goal_handle, target)
            if not aligned:
                return self._mission_failed(
                    goal_handle,
                    mission_id,
                    message,
                    index,
                    len(targets),
                    target,
                )

            self._publish_current_mission(
                mission_id,
                "analyzing",
                f"Analyzing image at {target.label}",
                target,
                index,
                len(targets),
            )
            analyzed, message = self._analyze(goal_handle, mission_id, target)
            if not analyzed:
                return self._mission_failed(
                    goal_handle,
                    mission_id,
                    message,
                    index,
                    len(targets),
                    target,
                )

            self._publish_scan_progress(
                target,
                index + 1,
                len(targets),
                "analysis_complete",
                message,
            )
            self._publish_current_mission(
                mission_id,
                "checkpoint_complete",
                f"Completed {target.label}: {message}",
                target,
                index + 1,
                len(targets),
            )
            self._publish_checkpoint_status(
                "arrived",
                f"Completed {target.label}",
                index + 1,
                target,
            )

        message = f"Mission complete: analyzed {len(targets)} checkpoints"
        self._publish_current_mission(
            mission_id,
            "complete",
            message,
            queue_index=len(targets),
            queue_total=len(targets),
        )
        self._publish_checkpoint_status("complete", message, len(targets))
        self._set_state("AUTONOMOUS_IDLE", message)
        return self._finish(goal_handle, True, message)

    def _navigate(
        self,
        goal_handle,
        target: MissionTarget,
    ) -> tuple[bool, str]:
        if not self._nav_client.wait_for_server(
            timeout_sec=self._double_parameter("action_server_timeout_sec")
        ):
            return False, "Nav2 NavigateToPose action is unavailable"

        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(target.pose)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        sent, nav_goal = self._send_goal(
            goal_handle,
            self._nav_client,
            goal,
            lambda msg: self._publish_navigation_feedback(goal_handle, msg),
        )
        if not sent:
            return False, str(nav_goal)

        self._active_nav_goal = nav_goal
        completed, wrapped_result = self._wait_for_future(
            goal_handle,
            nav_goal.get_result_async(),
            self._double_parameter("navigation_timeout_sec"),
        )
        self._active_nav_goal = None
        if not completed:
            return False, str(wrapped_result)
        if wrapped_result is None or int(wrapped_result.status) != 4:
            status = (
                "none"
                if wrapped_result is None
                else wrapped_result.status
            )
            return False, f"Nav2 failed with status {status}"
        return True, f"Arrived at {target.label}"

    def _align(
        self,
        goal_handle,
        target: MissionTarget,
    ) -> tuple[bool, str]:
        if not self._scan_client.wait_for_server(
            timeout_sec=self._double_parameter("action_server_timeout_sec")
        ):
            return False, "Scan-position alignment action is unavailable"

        goal = ExecuteScanPosition.Goal()
        goal.scan_position = target.scan_position
        goal.side = target.side
        goal.target_distance_m = (
            target.target_distance_m
            if target.target_distance_m > 0.0
            else self._double_parameter("scan_target_distance_m")
        )
        goal.hold_duration_sec = self._double_parameter(
            "scan_hold_duration_sec"
        )
        goal.dry_run = self._bool_parameter("scan_position_dry_run")
        sent, scan_goal = self._send_goal(
            goal_handle,
            self._scan_client,
            goal,
            lambda msg: self._publish_feedback(
                goal_handle,
                msg.feedback.message or f"Alignment: {msg.feedback.phase}",
                0.6,
            ),
        )
        if not sent:
            return False, str(scan_goal)

        self._active_scan_goal = scan_goal
        completed, wrapped_result = self._wait_for_future(
            goal_handle,
            scan_goal.get_result_async(),
            None,
        )
        self._active_scan_goal = None
        if not completed or wrapped_result is None:
            return False, str(wrapped_result or "Alignment returned no result")
        result = wrapped_result.result
        return bool(result.success), result.message or "Alignment complete"

    def _analyze(
        self,
        goal_handle,
        mission_id: str,
        target: MissionTarget,
    ) -> tuple[bool, str]:
        if not self._analysis_client.wait_for_server(
            timeout_sec=self._double_parameter("action_server_timeout_sec")
        ):
            return False, "Plant-analysis action is unavailable"

        goal = AnalyzePlantScan.Goal()
        goal.scan_position = target.scan_position
        goal.side = target.side
        goal.mission_id = mission_id
        goal.request_id = (
            f"{mission_id}:"
            f"{target.scan_position.scan_position_id}:"
            f"{target.side}"
        )
        goal.timeout_sec = self._double_parameter(
            "plant_analysis_timeout_sec"
        )
        goal.dry_run = self._bool_parameter("plant_analysis_dry_run")
        sent, analysis_goal = self._send_goal(
            goal_handle,
            self._analysis_client,
            goal,
            lambda msg: self._publish_feedback(
                goal_handle,
                msg.feedback.message or f"Analysis: {msg.feedback.phase}",
                0.8,
            ),
        )
        if not sent:
            return False, str(analysis_goal)

        self._active_analysis_goal = analysis_goal
        completed, wrapped_result = self._wait_for_future(
            goal_handle,
            analysis_goal.get_result_async(),
            None,
        )
        self._active_analysis_goal = None
        if not completed or wrapped_result is None:
            return False, str(wrapped_result or "Analysis returned no result")
        result = wrapped_result.result
        if result.success:
            self._latest_plant_health = result.plant_health
        return bool(result.success), result.message or "Analysis complete"

    def _send_goal(self, parent_goal, client, goal, feedback_callback):
        future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )
        completed, action_goal = self._wait_for_future(
            parent_goal,
            future,
            5.0,
        )
        if not completed:
            return False, action_goal
        if action_goal is None or not action_goal.accepted:
            return False, "Action goal was rejected"
        return True, action_goal

    def _wait_for_future(self, goal_handle, future, timeout_sec: float | None):
        deadline = (
            None
            if timeout_sec is None
            else time.monotonic() + float(timeout_sec)
        )
        while rclpy.ok() and not future.done():
            stop_message = self._stop_message(goal_handle)
            if stop_message is not None:
                return False, stop_message
            if deadline is not None and time.monotonic() >= deadline:
                self._request_stop("Action timed out")
                return False, "Action timed out"
            time.sleep(0.05)
        if not future.done():
            return False, "ROS shutdown"
        try:
            return True, future.result()
        except Exception as exc:
            return False, f"Action failed: {exc}"

    def _request_stop(self, message: str) -> None:
        self._stop_requested.set()
        self._state_message = message
        self._cmd_vel_pub.publish(Twist())
        for goal in (
            self._active_nav_goal,
            self._active_scan_goal,
            self._active_analysis_goal,
        ):
            if goal is not None:
                try:
                    goal.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(
                        f"Could not cancel child action: {exc}"
                    )

    def _stop_message(self, goal_handle) -> str | None:
        if goal_handle.is_cancel_requested:
            self._request_stop("Mission action cancelled")
            return "Mission action cancelled"
        if self._stop_requested.is_set():
            return self._state_message or "Mission stopped"
        return None

    def _load_route(
        self,
        annotations_file: str,
    ) -> tuple[list[MissionTarget], PoseStamped]:
        path = Path(annotations_file).expanduser()
        with path.open("r", encoding="utf-8") as stream:
            annotations = json.load(stream)

        home_pose = self._pose_from_dict(annotations["home_pose"])
        checkpoints = sorted(
            annotations.get("checkpoints", []),
            key=lambda item: int(
                item.get("order", item.get("checkpoint_id", 0))
            ),
        )
        targets = [
            target
            for checkpoint in checkpoints
            if (target := self._target_from_annotation(checkpoint)) is not None
        ]
        if not targets:
            raise RuntimeError(f"No valid checkpoints found in {path}")
        return targets, home_pose

    def _target_from_annotation(
        self,
        checkpoint: dict,
    ) -> MissionTarget | None:
        metadata = checkpoint.get("metadata", {})
        if not isinstance(metadata, dict):
            metadata = {}
        label = str(checkpoint.get("label", "")).strip()
        bed_id = str(
            checkpoint.get("bed_id", metadata.get("bed_id", ""))
        ).strip()
        side = str(
            checkpoint.get("side", metadata.get("side", ""))
        ).strip().lower()
        scan_id = str(
            checkpoint.get(
                "scan_position_id",
                metadata.get("scan_position_id", label),
            )
        ).strip()
        pose_data = checkpoint.get("pose")
        if (
            not bed_id
            or side not in ("a", "b")
            or not scan_id
            or not isinstance(pose_data, dict)
        ):
            return None

        pose = self._pose_from_dict(pose_data)
        scan_position = ScanPosition()
        scan_position.scan_position_id = scan_id
        scan_position.bed_id = bed_id
        scan_position.base_pose.x = pose.pose.position.x
        scan_position.base_pose.y = pose.pose.position.y
        scan_position.base_pose.theta = self._yaw_from_pose(pose)
        scan_position.order = int(
            checkpoint.get("order", metadata.get("order", 0))
        )
        scan_position.enabled = True
        raw_distance = checkpoint.get(
            "target_distance_m",
            metadata.get("target_distance_m"),
        )
        try:
            target_distance = (
                float(raw_distance)
                if raw_distance is not None
                else 0.0
            )
        except (TypeError, ValueError):
            target_distance = 0.0
        return MissionTarget(
            label=label or scan_id,
            scan_position=scan_position,
            side=side,
            pose=pose,
            target_distance_m=target_distance,
        )

    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        orientation = pose.pose.orientation
        return math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _pose_from_dict(self, data: dict) -> PoseStamped:
        position = data.get("position", {})
        orientation = data.get("orientation", {})
        pose = PoseStamped()
        pose.header.frame_id = str(data.get("frame_id") or "map")
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return pose

    def _target_matches(
        self,
        target_filter: str,
        target: MissionTarget,
    ) -> bool:
        normalized = target_filter.strip().lower()
        if not normalized or normalized == "all":
            return True
        return normalized in (
            target.label.lower(),
            target.scan_position.bed_id.lower(),
            target.scan_position.scan_position_id.lower(),
            f"{target.scan_position.bed_id}:{target.side}".lower(),
        )

    def _on_timer(self) -> None:
        self._publish_task_status()
        if self._initial_pose_remaining <= 0:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._home_pose.header.frame_id or "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = copy.deepcopy(self._home_pose.pose)
        msg.pose.covariance[0] = self._double_parameter(
            "initial_pose_covariance_xy"
        )
        msg.pose.covariance[7] = self._double_parameter(
            "initial_pose_covariance_xy"
        )
        msg.pose.covariance[35] = self._double_parameter(
            "initial_pose_covariance_yaw"
        )
        self._initial_pose_pub.publish(msg)
        self._initial_pose_remaining -= 1

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose

    def _publish_navigation_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = max(0.0, float(feedback.distance_remaining))
        self._publish_feedback(
            goal_handle,
            f"Nav2 driving: {distance:.2f} m remaining",
            max(0.05, min(0.55, 0.55 / (1.0 + distance))),
        )

    def _publish_feedback(
        self,
        goal_handle,
        message: str,
        progress: float,
    ) -> None:
        feedback = ExecuteBehavior.Feedback()
        feedback.current_step = message
        feedback.progress = max(0.0, min(1.0, float(progress)))
        goal_handle.publish_feedback(feedback)

    def _publish_task_status(self) -> None:
        msg = TaskStatus()
        msg.current_state = self._state
        msg.active = self._mission_reserved
        msg.error = self._state == "ERROR"
        msg.message = self._state_message
        self._task_status_pub.publish(msg)

    def _set_state(self, state: str, message: str) -> None:
        self._state = state
        self._state_message = message
        self._publish_task_status()

    def _publish_current_mission(
        self,
        mission_id: str,
        phase: str,
        message: str,
        target: MissionTarget | None = None,
        queue_index: int = 0,
        queue_total: int = 0,
        error: bool = False,
    ) -> None:
        msg = CurrentMission()
        msg.mission_id = mission_id
        msg.phase = phase
        msg.queue_index = max(0, int(queue_index))
        msg.queue_total = max(0, int(queue_total))
        msg.current_pose = self._latest_pose
        msg.latest_plant_health = self._latest_plant_health
        msg.error = error
        msg.message = message
        if target is not None:
            msg.active_bed_id = target.scan_position.bed_id
            msg.active_side = target.side
            msg.active_scan_position_id = target.scan_position.scan_position_id
            msg.target_pose = target.pose
        self._current_mission_pub.publish(msg)

    def _publish_scan_progress(
        self,
        target: MissionTarget,
        scan_index: int,
        scan_total: int,
        detection_status: str,
        message: str,
        error: bool = False,
    ) -> None:
        msg = ScanProgress()
        msg.active_bed_id = target.scan_position.bed_id
        msg.active_scan_position_id = target.scan_position.scan_position_id
        msg.scan_index = max(0, int(scan_index))
        msg.scan_total = max(0, int(scan_total))
        msg.detection_status = detection_status
        msg.latest_plant_health = self._latest_plant_health
        msg.error = error
        msg.message = message
        self._scan_progress_pub.publish(msg)

    def _publish_checkpoint_status(
        self,
        event: str,
        message: str,
        next_index: int,
        target: MissionTarget | None = None,
        error: bool = False,
    ) -> None:
        next_target = (
            self._targets[next_index]
            if 0 <= next_index < len(self._targets)
            else None
        )
        payload = {
            "event": event,
            "state": self._state.lower(),
            "message": message,
            "error": error,
            "next_index": int(next_index),
            "route_length": len(self._targets),
            "next_target": self._target_payload(next_target),
            "active_target": (
                self._target_payload(target)
                if event in ("navigating", "started")
                else None
            ),
            "arrived_target": (
                self._target_payload(target) if event == "arrived" else None
            ),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._checkpoint_status_pub.publish(msg)

    def _target_payload(self, target: MissionTarget | None):
        if target is None:
            return None
        return {
            "label": target.label,
            "metadata": {
                "bed_id": target.scan_position.bed_id,
                "side": target.side,
                "scan_position_id": target.scan_position.scan_position_id,
                "order": int(target.scan_position.order),
                "target_distance_m": target.target_distance_m,
            },
        }

    def _mission_failed(
        self,
        goal_handle,
        mission_id: str,
        message: str,
        queue_index: int = 0,
        queue_total: int = 0,
        target: MissionTarget | None = None,
    ):
        self._request_stop(message)
        self._publish_current_mission(
            mission_id,
            "failed",
            message,
            target,
            queue_index,
            queue_total,
            error=True,
        )
        if target is not None:
            self._publish_scan_progress(
                target,
                queue_index,
                queue_total,
                "failed",
                message,
                error=True,
            )
        self._publish_checkpoint_status(
            "failed",
            message,
            queue_index,
            target,
            error=True,
        )
        self._set_state("ERROR", message)
        return self._finish(goal_handle, False, message)

    def _mission_stopped(self, goal_handle, mission_id: str, message: str):
        self._publish_current_mission(mission_id, "cancelled", message)
        self._publish_checkpoint_status("cancelled", message, 0)
        self._set_state("AUTONOMOUS_IDLE", message)
        result = ExecuteBehavior.Result()
        result.success = False
        result.message = message
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _finish(self, goal_handle, success: bool, message: str):
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
    node = SimpleMissionManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._request_stop("Node shutting down")
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

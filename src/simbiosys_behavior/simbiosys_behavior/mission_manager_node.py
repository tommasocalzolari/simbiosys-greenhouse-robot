import json
import math
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from simbiosys_behavior.state_machine import MissionStateMachine
from simbiosys_interfaces.action import (
    AnalyzePlantScan,
    ExecuteBedSideScan,
    ExecuteBehavior,
    ExecuteScanPosition,
)
from simbiosys_interfaces.msg import (
    BehaviorType,
    CurrentMission,
    HarvestStatus,
    NavigationStatus,
    PlantHealth,
    ScanProgress,
    ScanPosition,
    TaskStatus,
)
from simbiosys_interfaces.srv import (
    GetHarvestEnabled,
    SetHarvestEnabled,
    SetRobotMode,
)

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - depends on installed Nav2 packages.
    NavigateToPose = None


@dataclass
class TopicHealth:
    scan_seen: bool = False
    odom_seen: bool = False
    map_seen: bool = False
    amcl_seen: bool = False
    plan_seen: bool = False


@dataclass
class QueuedScanPosition:
    scan_position: ScanPosition
    side: str


class FailureCode:
    """Shared behavior failure labels used in action results and status text."""

    PRECONDITION_FAILED = "PRECONDITION_FAILED"
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    HARVEST_DISABLED = "HARVEST_DISABLED"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"


class MissionManagerNode(Node):
    """Laptop-side coordinator for SimBioSys behavior requests.

    This is intentionally a thin orchestration layer. It validates that the
    expected robot/ROS surfaces are available, publishes typed behavior status,
    and delegates actual motion/navigation to the existing stacks.
    """

    def __init__(self) -> None:
        super().__init__("mission_manager_node")
        self._callback_group = ReentrantCallbackGroup()
        self._state_machine = MissionStateMachine()
        self._topic_health = TopicHealth()
        self._topic_lock = threading.Lock()
        self._active_nav_goal_handle = None
        self._active_bed_side_goal_handle = None
        self._active_scan_position_goal_handle = None
        self._active_plant_analysis_goal_handle = None
        self._latest_pose = PoseStamped()
        self._latest_path = Path()
        self._latest_plant_health = PlantHealth()
        self._latest_plant_health_time = 0.0

        self.declare_parameter("harvest_enabled", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/mirte_base_controller/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("nav_plan_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter(
            "bed_side_scan_action_name",
            "simbiosys/execute_bed_side_scan",
        )
        self.declare_parameter(
            "scan_position_action_name",
            "simbiosys/execute_scan_position",
        )
        self.declare_parameter(
            "plant_analysis_action_name",
            "simbiosys/analyze_plant_scan",
        )
        self.declare_parameter("nav2_server_timeout_sec", 2.0)
        self.declare_parameter("bed_side_scan_server_timeout_sec", 0.5)
        self.declare_parameter("scan_position_server_timeout_sec", 0.5)
        self.declare_parameter("plant_analysis_server_timeout_sec", 0.5)
        self.declare_parameter("require_localization_for_navigation", True)
        self.declare_parameter("bed_side_scan_dry_run", True)
        self.declare_parameter("scan_position_dry_run", True)
        self.declare_parameter("plant_analysis_dry_run", False)
        self.declare_parameter("min_flowers_per_bed_side", 1)
        self.declare_parameter("home_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_initial_pose_on_mission_start", True)
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("initial_pose_covariance_xy", 0.25)
        self.declare_parameter("initial_pose_covariance_yaw", 0.0685)
        self.declare_parameter("scan_positions", [""])
        self.declare_parameter("mission_localization_timeout_sec", 10.0)
        self.declare_parameter("scan_position_target_distance_m", 0.35)
        self.declare_parameter("scan_position_hold_duration_sec", 1.0)
        self.declare_parameter("plant_health_topic", "simbiosys/plant_health")
        self.declare_parameter("plant_analysis_timeout_sec", 5.0)
        self.declare_parameter("return_home_when_queue_empty", True)

        self._harvest_enabled = bool(self.get_parameter("harvest_enabled").value)

        self._status_publisher = self.create_publisher(
            TaskStatus,
            "simbiosys/task_status",
            10,
        )
        self._navigation_status_publisher = self.create_publisher(
            NavigationStatus,
            "simbiosys/navigation_status",
            10,
        )
        self._scan_progress_publisher = self.create_publisher(
            ScanProgress,
            "simbiosys/scan_progress",
            10,
        )
        self._current_mission_publisher = self.create_publisher(
            CurrentMission,
            "simbiosys/current_mission",
            10,
        )
        self._harvest_status_publisher = self.create_publisher(
            HarvestStatus,
            "simbiosys/harvest_status",
            10,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self._string_parameter("initial_pose_topic"),
            10,
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self._string_parameter("cmd_vel_topic"),
            10,
        )

        self._subscribe_to_health_topics()

        self.create_service(
            SetRobotMode,
            "simbiosys/set_robot_mode",
            self._on_set_robot_mode,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetHarvestEnabled,
            "simbiosys/set_harvest_enabled",
            self._on_set_harvest_enabled,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetHarvestEnabled,
            "simbiosys/get_harvest_enabled",
            self._on_get_harvest_enabled,
            callback_group=self._callback_group,
        )

        self._nav2_client = None
        if NavigateToPose is not None:
            self._nav2_client = ActionClient(
                self,
                NavigateToPose,
                self._string_parameter("nav2_action_name"),
                callback_group=self._callback_group,
            )
        self._bed_side_scan_client = ActionClient(
            self,
            ExecuteBedSideScan,
            self._string_parameter("bed_side_scan_action_name"),
            callback_group=self._callback_group,
        )
        self._scan_position_client = ActionClient(
            self,
            ExecuteScanPosition,
            self._string_parameter("scan_position_action_name"),
            callback_group=self._callback_group,
        )
        self._plant_analysis_client = ActionClient(
            self,
            AnalyzePlantScan,
            self._string_parameter("plant_analysis_action_name"),
            callback_group=self._callback_group,
        )

        self._execute_behavior_server = ActionServer(
            self,
            ExecuteBehavior,
            "simbiosys/execute_behavior",
            execute_callback=self._on_execute_behavior,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            "Mission manager ready: ExecuteBehavior, robot mode, harvest flag, "
            "and typed behavior status publishers are available"
        )

    def _subscribe_to_health_topics(self) -> None:
        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            lambda _msg: self._mark_seen("scan"),
            10,
        )
        self.create_subscription(
            Odometry,
            self._string_parameter("odom_topic"),
            self._on_odom,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self._string_parameter("map_topic"),
            lambda _msg: self._mark_seen("map"),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._string_parameter("amcl_pose_topic"),
            self._on_amcl_pose,
            10,
        )
        self.create_subscription(
            Path,
            self._string_parameter("nav_plan_topic"),
            self._on_nav_plan,
            10,
        )
        self.create_subscription(
            PlantHealth,
            self._string_parameter("plant_health_topic"),
            self._on_plant_health,
            10,
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _parameter_value(self, name: str):
        return self.get_parameter(name).value

    def _float_array_parameter(self, name: str) -> list[float]:
        value = self._parameter_value(name)
        if isinstance(value, (list, tuple)):
            return [float(item) for item in value]
        if isinstance(value, str):
            return [float(item.strip()) for item in value.split(",") if item.strip()]
        return []

    def _string_array_parameter(self, name: str) -> list[str]:
        value = self._parameter_value(name)
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        if isinstance(value, str) and value.strip():
            return [value]
        return []

    def _mark_seen(self, key: str) -> None:
        with self._topic_lock:
            if key == "scan":
                self._topic_health.scan_seen = True
            elif key == "odom":
                self._topic_health.odom_seen = True
            elif key == "map":
                self._topic_health.map_seen = True
            elif key == "amcl":
                self._topic_health.amcl_seen = True
            elif key == "plan":
                self._topic_health.plan_seen = True

    def _on_odom(self, msg: Odometry) -> None:
        self._mark_seen("odom")
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._mark_seen("amcl")
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose

    def _on_nav_plan(self, msg: Path) -> None:
        self._mark_seen("plan")
        self._latest_path = msg

    def _on_plant_health(self, msg: PlantHealth) -> None:
        self._latest_plant_health = msg
        self._latest_plant_health_time = time.monotonic()

    def _health_snapshot(self) -> TopicHealth:
        with self._topic_lock:
            return TopicHealth(
                scan_seen=self._topic_health.scan_seen,
                odom_seen=self._topic_health.odom_seen,
                map_seen=self._topic_health.map_seen,
                amcl_seen=self._topic_health.amcl_seen,
                plan_seen=self._topic_health.plan_seen,
            )

    def _on_timer(self) -> None:
        state = self._state_machine.step()

        status = TaskStatus()
        status.current_state = state.value
        status.active = state.value not in ("ERROR", "WAIT_FOR_OPERATOR")
        status.error = state.value == "ERROR"
        status.message = (
            f"Mission manager state={state.value}; "
            f"harvest_enabled={self._harvest_enabled}"
        )

        self._status_publisher.publish(status)

    def _on_set_robot_mode(self, request, response):
        success, message = self._state_machine.set_mode(request.mode)
        response.success = success
        response.message = message
        self.get_logger().info(message)
        return response

    def _on_set_harvest_enabled(self, request, response):
        self._harvest_enabled = bool(request.enabled)
        self.set_parameters(
            [
                Parameter(
                    "harvest_enabled",
                    Parameter.Type.BOOL,
                    self._harvest_enabled,
                )
            ]
        )
        response.success = True
        response.enabled = self._harvest_enabled
        response.message = f"harvest_enabled set to {self._harvest_enabled}"
        self._publish_harvest_status("configured", response.message)
        self.get_logger().info(response.message)
        return response

    def _on_get_harvest_enabled(self, _request, response):
        response.enabled = self._harvest_enabled
        response.message = f"harvest_enabled is {self._harvest_enabled}"
        return response

    def _goal_callback(
        self,
        goal_request: ExecuteBehavior.Goal,
    ) -> GoalResponse:
        behavior = goal_request.behavior.type
        if behavior not in self._behavior_names():
            self.get_logger().warn(f"Rejecting unknown behavior type {behavior}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for active behavior")
        self._cancel_active_work()
        return CancelResponse.ACCEPT

    def _on_execute_behavior(self, goal_handle):
        behavior = goal_handle.request.behavior.type
        behavior_name = self._behavior_name(behavior)
        target_id = goal_handle.request.target_id

        self._publish_feedback(goal_handle, f"Accepted {behavior_name}", 0.05)
        self.get_logger().info(
            f"ExecuteBehavior accepted: behavior={behavior_name}, "
            f"target_id='{target_id}'"
        )

        success, message = self._dispatch_behavior(goal_handle)

        result = ExecuteBehavior.Result()
        result.success = success
        result.message = message

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = f"{behavior_name} cancelled"
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(result.message)
        return result

    def _dispatch_behavior(self, goal_handle) -> tuple[bool, str]:
        behavior = goal_handle.request.behavior.type
        if behavior == BehaviorType.IDLE:
            return self._execute_idle(goal_handle)
        if behavior == BehaviorType.TELEOP:
            return self._execute_teleop(goal_handle)
        if behavior == BehaviorType.MAP:
            return self._execute_map(goal_handle)
        if behavior == BehaviorType.LOCALIZE:
            return self._execute_localize(goal_handle)
        if behavior == BehaviorType.NAVIGATE:
            return self._execute_navigate(goal_handle)
        if behavior == BehaviorType.INSPECT_BED:
            return self._execute_inspect_bed(goal_handle)
        if behavior == BehaviorType.INSPECT_FLOWER:
            return self._execute_inspect_flower(goal_handle)
        if behavior == BehaviorType.HARVEST:
            return self._execute_harvest(goal_handle)
        if behavior == BehaviorType.ARM_TEST:
            return self._set_mode_for_behavior(behavior)
        return False, f"Unknown behavior type {behavior}"

    def _execute_idle(self, goal_handle) -> tuple[bool, str]:
        self._publish_feedback(goal_handle, "Stopping active work", 0.4)
        self._cancel_active_work()
        success, message = self._set_mode_for_behavior(BehaviorType.IDLE)
        self._publish_feedback(goal_handle, message, 1.0 if success else 0.0)
        return success, f"IDLE requested: {message}"

    def _execute_teleop(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.TELEOP)
        self._publish_feedback(goal_handle, message, 1.0 if success else 0.0)
        return success, f"TELEOP requested: {message}"

    def _execute_map(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.MAP)
        if not success:
            return False, message
        health = self._health_snapshot()
        missing = self._missing_topics(health, ["scan", "odom", "map"])
        progress = 0.35
        if missing:
            warning = (
                "Mapping mode set, but waiting for required mapping topics: "
                + ", ".join(missing)
            )
            self._publish_feedback(goal_handle, warning, progress)
            return True, warning
        self._publish_feedback(goal_handle, "Mapping topics are available", 1.0)
        return True, "MAP workflow ready: drive with teleop and finish from UI"

    def _execute_localize(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.LOCALIZE)
        if not success:
            return False, message
        health = self._health_snapshot()
        missing = self._missing_topics(health, ["scan", "odom", "map"])
        if missing:
            message = "Localization waiting for topics: " + ", ".join(missing)
            self._publish_feedback(goal_handle, message, 0.4)
            return False, message
        localized = health.amcl_seen
        message = (
            "Localization pose received"
            if localized
            else "Localization inputs ready; waiting for /amcl_pose"
        )
        self._publish_feedback(goal_handle, message, 1.0 if localized else 0.7)
        return localized, message

    def _execute_navigate(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.NAVIGATE)
        if not success:
            return False, message

        target_pose = goal_handle.request.target_pose
        if not self._pose_is_finite(target_pose):
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "target_pose contains non-finite values"
            )
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        return self._navigate_to_pose(
            goal_handle,
            target_pose,
            "NAVIGATE",
            set_idle_on_success=True,
        )

    def _navigate_to_pose(
        self,
        goal_handle,
        target_pose,
        label: str,
        set_idle_on_success: bool = False,
    ) -> tuple[bool, str]:
        health = self._health_snapshot()
        required = ["odom", "map"]
        if self._bool_parameter("require_localization_for_navigation"):
            required.append("amcl")
        missing = self._missing_topics(health, required)
        if missing:
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: navigation waiting for "
                + ", ".join(missing)
            )
            self._publish_feedback(goal_handle, message, 0.1)
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        if self._nav2_client is None:
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "nav2_msgs NavigateToPose is not available"
            )
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        self._publish_feedback(goal_handle, "Waiting for Nav2 NavigateToPose", 0.2)
        if not self._nav2_client.wait_for_server(
            timeout_sec=self._double_parameter("nav2_server_timeout_sec")
        ):
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "Nav2 NavigateToPose action is unavailable"
            )
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(target_pose)
        self._publish_navigation_status("planning", "Sent Nav2 goal", goal.pose)
        self._publish_feedback(goal_handle, "Sent Nav2 goal", 0.35)

        send_goal_future = self._nav2_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_nav2_feedback(
                goal_handle,
                feedback,
            ),
        )
        send_deadline = time.monotonic() + 5.0
        while rclpy.ok() and not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "NAVIGATE cancelled"
            if time.monotonic() >= send_deadline:
                message = (
                    f"{FailureCode.PRECONDITION_FAILED}: timed out sending Nav2 goal"
                )
                self._publish_navigation_status("failed", message, goal.pose, error=True)
                return False, message
            time.sleep(0.05)
        nav_goal_handle = send_goal_future.result()
        if nav_goal_handle is None or not nav_goal_handle.accepted:
            message = f"{FailureCode.PLANNING_FAILED}: Nav2 rejected the goal"
            self._publish_navigation_status("failed", message, goal.pose, error=True)
            return False, message

        self._active_nav_goal_handle = nav_goal_handle
        result_future = nav_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "NAVIGATE cancelled"
            time.sleep(0.1)

        result = result_future.result()
        self._active_nav_goal_handle = None
        if result is None:
            message = f"{FailureCode.EXECUTION_FAILED}: Nav2 returned no result"
            self._publish_navigation_status("failed", message, goal.pose, error=True)
            return False, message

        status = int(result.status)
        if status == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            message = f"{label} completed"
            self._publish_feedback(goal_handle, message, 1.0)
            self._publish_navigation_status("arrived", message, goal.pose)
            if set_idle_on_success:
                self._set_mode_for_behavior(BehaviorType.IDLE)
            return True, message

        message = f"{FailureCode.EXECUTION_FAILED}: Nav2 finished with status {status}"
        self._publish_navigation_status("failed", message, goal.pose, error=True)
        return False, message

    def _execute_inspect_bed(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.INSPECT_BED)
        if not success:
            return False, message
        target_id = goal_handle.request.target_id.strip()
        mission_id = f"scan-{int(time.time())}"
        queue = self._scan_queue_for_target(target_id)
        if not queue:
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "INSPECT_BED found no configured scan positions"
            )
            self._publish_current_mission(mission_id, "failed", message, error=True)
            return False, message

        self._publish_initial_pose_for_home()
        self._publish_current_mission(
            mission_id,
            "localizing",
            "Published home initial pose; waiting for localization inputs",
            queue_index=0,
            queue_total=len(queue),
        )
        localization_ok, localization_message = self._wait_for_mission_localization(
            goal_handle,
            mission_id,
            len(queue),
        )
        if not localization_ok:
            return False, localization_message

        skipped = 0
        for index, queued_position in enumerate(queue):
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"

            scan_position = queued_position.scan_position
            target_pose = self._pose_from_scan_position(scan_position)
            self._publish_current_mission(
                mission_id,
                "navigating",
                f"Navigating to {scan_position.scan_position_id}",
                queued_position,
                index,
                len(queue),
                target_pose,
            )
            nav_success, nav_message = self._navigate_to_pose(
                goal_handle,
                target_pose.pose,
                f"Navigate to {scan_position.scan_position_id}",
                set_idle_on_success=False,
            )
            if not nav_success:
                skipped += 1
                self._publish_current_mission(
                    mission_id,
                    "skipped",
                    nav_message,
                    queued_position,
                    index,
                    len(queue),
                    target_pose,
                    error=True,
                )
                continue

            self._publish_current_mission(
                mission_id,
                "aligning",
                f"Aligning at {scan_position.scan_position_id}",
                queued_position,
                index,
                len(queue),
                target_pose,
            )
            scan_success, scan_message = self._execute_scan_position(
                goal_handle,
                queued_position,
            )
            if not scan_success:
                skipped += 1
                self._publish_current_mission(
                    mission_id,
                    "skipped",
                    scan_message,
                    queued_position,
                    index,
                    len(queue),
                    target_pose,
                    error=True,
                )
                continue

            self._publish_current_mission(
                mission_id,
                "scanning",
                f"Requesting plant analysis at {scan_position.scan_position_id}",
                queued_position,
                index,
                len(queue),
                target_pose,
            )
            plant_ok, plant_message = self._execute_plant_analysis(
                goal_handle,
                mission_id,
                queued_position,
            )
            if not plant_ok:
                skipped += 1
                self._publish_current_mission(
                    mission_id,
                    "skipped",
                    plant_message,
                    queued_position,
                    index,
                    len(queue),
                    target_pose,
                    error=True,
                )
                continue

            self._publish_current_mission(
                mission_id,
                "completed_scan_position",
                f"Completed {scan_position.scan_position_id}",
                queued_position,
                index,
                len(queue),
                target_pose,
            )
            self._publish_scan_progress(
                f"Completed {scan_position.scan_position_id}",
                active_bed_id=scan_position.bed_id,
                active_scan_position_id=scan_position.scan_position_id,
                detection_status="plant_analysis_received",
                latest_plant_health=self._latest_plant_health,
            )

        if self._bool_parameter("return_home_when_queue_empty"):
            home_pose = self._home_pose_stamped()
            self._publish_current_mission(
                mission_id,
                "returning_home",
                "Returning home",
                queue_index=len(queue),
                queue_total=len(queue),
                target_pose=home_pose,
            )
            home_success, home_message = self._navigate_to_pose(
                goal_handle,
                home_pose.pose,
                "Return home",
                set_idle_on_success=False,
            )
            if not home_success:
                self._publish_current_mission(
                    mission_id,
                    "failed",
                    home_message,
                    queue_index=len(queue),
                    queue_total=len(queue),
                    target_pose=home_pose,
                    error=True,
                )
                return False, home_message

        message = (
            f"INSPECT_BED queued mission complete; "
            f"completed={len(queue) - skipped}, skipped={skipped}"
        )
        self._publish_current_mission(
            mission_id,
            "complete",
            message,
            queue_index=len(queue),
            queue_total=len(queue),
        )
        self._set_mode_for_behavior(BehaviorType.IDLE)
        return True, message

    def _wait_for_mission_localization(
        self,
        goal_handle,
        mission_id: str,
        queue_total: int,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + self._double_parameter(
            "mission_localization_timeout_sec"
        )
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            health = self._health_snapshot()
            missing = self._missing_topics(health, ["odom", "map", "amcl"])
            if not missing:
                message = "Localization ready"
                self._publish_feedback(goal_handle, message, 0.1)
                self._publish_current_mission(
                    mission_id,
                    "localized",
                    message,
                    queue_index=0,
                    queue_total=queue_total,
                )
                return True, message
            message = "Mission localization waiting for " + ", ".join(missing)
            self._publish_feedback(goal_handle, message, 0.05)
            self._publish_current_mission(
                mission_id,
                "localizing",
                message,
                queue_index=0,
                queue_total=queue_total,
            )
            if time.monotonic() >= deadline:
                message = f"{FailureCode.PRECONDITION_FAILED}: {message}"
                self._publish_current_mission(
                    mission_id,
                    "failed",
                    message,
                    queue_index=0,
                    queue_total=queue_total,
                    error=True,
                )
                return False, message
            time.sleep(0.1)

    def _execute_scan_position(
        self,
        goal_handle,
        queued_position: QueuedScanPosition,
    ) -> tuple[bool, str]:
        timeout_sec = self._double_parameter("scan_position_server_timeout_sec")
        if not self._scan_position_client.wait_for_server(timeout_sec=timeout_sec):
            return (
                False,
                f"{FailureCode.PRECONDITION_FAILED}: scan-position controller action is unavailable",
            )

        goal = ExecuteScanPosition.Goal()
        goal.scan_position = queued_position.scan_position
        goal.side = queued_position.side
        goal.target_distance_m = float(
            self._double_parameter("scan_position_target_distance_m")
        )
        goal.hold_duration_sec = float(
            self._double_parameter("scan_position_hold_duration_sec")
        )
        goal.dry_run = self._bool_parameter("scan_position_dry_run")

        send_goal_future = self._scan_position_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_scan_position_feedback(
                goal_handle,
                feedback,
            ),
        )
        send_deadline = time.monotonic() + 5.0
        while rclpy.ok() and not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            if time.monotonic() >= send_deadline:
                return (
                    False,
                    f"{FailureCode.PRECONDITION_FAILED}: timed out sending scan-position goal",
                )
            time.sleep(0.05)

        scan_goal_handle = send_goal_future.result()
        if scan_goal_handle is None or not scan_goal_handle.accepted:
            return False, f"{FailureCode.PLANNING_FAILED}: scan-position goal rejected"

        self._active_scan_position_goal_handle = scan_goal_handle
        result_future = scan_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            time.sleep(0.1)

        self._active_scan_position_goal_handle = None
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, f"{FailureCode.EXECUTION_FAILED}: scan-position returned no result"

        result = wrapped_result.result
        if result.success:
            return True, result.message or "scan position aligned"
        return False, result.message or f"{FailureCode.EXECUTION_FAILED}: scan-position failed"

    def _execute_plant_analysis(
        self,
        goal_handle,
        mission_id: str,
        queued_position: QueuedScanPosition,
    ) -> tuple[bool, str]:
        timeout_sec = self._double_parameter("plant_analysis_server_timeout_sec")
        if not self._plant_analysis_client.wait_for_server(timeout_sec=timeout_sec):
            return (
                False,
                f"{FailureCode.PRECONDITION_FAILED}: plant analysis action is unavailable",
            )

        scan_position = queued_position.scan_position
        goal = AnalyzePlantScan.Goal()
        goal.scan_position = scan_position
        goal.side = queued_position.side
        goal.mission_id = mission_id
        goal.request_id = (
            f"{mission_id}:{scan_position.scan_position_id}:{queued_position.side}"
        )
        goal.timeout_sec = float(self._double_parameter("plant_analysis_timeout_sec"))
        goal.dry_run = self._bool_parameter("plant_analysis_dry_run")

        send_goal_future = self._plant_analysis_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_plant_analysis_feedback(
                goal_handle,
                feedback,
            ),
        )
        send_deadline = time.monotonic() + 5.0
        while rclpy.ok() and not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            if time.monotonic() >= send_deadline:
                return (
                    False,
                    f"{FailureCode.PRECONDITION_FAILED}: timed out sending plant analysis goal",
                )
            time.sleep(0.05)

        analysis_goal_handle = send_goal_future.result()
        if analysis_goal_handle is None or not analysis_goal_handle.accepted:
            return False, f"{FailureCode.PLANNING_FAILED}: plant analysis goal rejected"

        self._active_plant_analysis_goal_handle = analysis_goal_handle
        result_future = analysis_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            time.sleep(0.1)

        self._active_plant_analysis_goal_handle = None
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, f"{FailureCode.EXECUTION_FAILED}: plant analysis returned no result"

        result = wrapped_result.result
        if not result.success:
            return False, result.message or f"{FailureCode.EXECUTION_FAILED}: plant analysis failed"

        self._latest_plant_health = result.plant_health
        self._latest_plant_health_time = time.monotonic()
        return True, result.message or "plant analysis completed"

    def _scan_queue_for_target(self, target_id: str) -> list[QueuedScanPosition]:
        entries = self._parse_scan_positions()
        normalized_target = target_id.strip()
        if not normalized_target or normalized_target.lower() == "all":
            return entries
        parsed_target = self._parse_bed_side_target(normalized_target)
        if parsed_target is None:
            return []
        bed_id, side = parsed_target
        return [
            entry
            for entry in entries
            if entry.scan_position.bed_id == bed_id and entry.side == side
        ]

    def _parse_scan_positions(self) -> list[QueuedScanPosition]:
        entries: list[QueuedScanPosition] = []
        for index, raw_entry in enumerate(self._string_array_parameter("scan_positions")):
            parsed = self._parse_scan_position_entry(raw_entry, index)
            if parsed is not None:
                entries.append(parsed)
        entries.sort(key=lambda entry: int(entry.scan_position.order))
        return entries

    def _parse_scan_position_entry(
        self,
        raw_entry: str,
        index: int,
    ) -> QueuedScanPosition | None:
        raw_entry = raw_entry.strip()
        if not raw_entry:
            return None
        try:
            data = json.loads(raw_entry)
        except json.JSONDecodeError:
            data = None

        if isinstance(data, dict):
            scan_id = str(data.get("scan_position_id") or data.get("id") or f"scan_{index}")
            bed_id = str(data.get("bed_id") or "")
            side = str(data.get("side") or "").lower()
            x = float(data.get("x", data.get("base_x", 0.0)))
            y = float(data.get("y", data.get("base_y", 0.0)))
            yaw = float(data.get("yaw", data.get("theta", 0.0)))
            order = int(data.get("order", index))
            enabled = bool(data.get("enabled", True))
        else:
            parts = [part.strip() for part in raw_entry.split(",")]
            if len(parts) < 6:
                self.get_logger().warning(
                    f"Ignoring malformed scan_positions[{index}]: '{raw_entry}'"
                )
                return None
            scan_id, bed_id, side = parts[0], parts[1], parts[2].lower()
            x, y, yaw = float(parts[3]), float(parts[4]), float(parts[5])
            enabled = parts[6].lower() not in ("false", "0", "no") if len(parts) > 6 else True
            order = index

        if not enabled:
            return None
        if not scan_id or not bed_id or side not in ("a", "b"):
            self.get_logger().warning(
                f"Ignoring scan position with invalid id/bed/side: '{raw_entry}'"
            )
            return None

        scan_position = ScanPosition()
        scan_position.scan_position_id = scan_id
        scan_position.bed_id = bed_id
        scan_position.base_pose.x = x
        scan_position.base_pose.y = y
        scan_position.base_pose.theta = yaw
        scan_position.order = order
        scan_position.enabled = True
        return QueuedScanPosition(scan_position=scan_position, side=side)

    def _publish_initial_pose_for_home(self) -> None:
        if not self._bool_parameter("publish_initial_pose_on_mission_start"):
            return
        home_pose = self._home_pose_stamped()
        msg = PoseWithCovarianceStamped()
        msg.header = home_pose.header
        msg.pose.pose = home_pose.pose
        msg.pose.covariance[0] = self._double_parameter("initial_pose_covariance_xy")
        msg.pose.covariance[7] = self._double_parameter("initial_pose_covariance_xy")
        msg.pose.covariance[35] = self._double_parameter("initial_pose_covariance_yaw")
        self._initial_pose_publisher.publish(msg)

    def _home_pose_stamped(self) -> PoseStamped:
        pose_values = self._float_array_parameter("home_pose")
        while len(pose_values) < 3:
            pose_values.append(0.0)
        return self._pose_stamped_from_xy_yaw(
            pose_values[0],
            pose_values[1],
            pose_values[2],
        )

    def _pose_from_scan_position(self, scan_position: ScanPosition) -> PoseStamped:
        return self._pose_stamped_from_xy_yaw(
            scan_position.base_pose.x,
            scan_position.base_pose.y,
            scan_position.base_pose.theta,
        )

    def _pose_stamped_from_xy_yaw(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _execute_bed_side_scan(
        self,
        goal_handle,
        bed_id: str,
        side: str,
    ) -> tuple[bool, str]:
        timeout_sec = self._double_parameter("bed_side_scan_server_timeout_sec")
        if not self._bed_side_scan_client.wait_for_server(timeout_sec=timeout_sec):
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "bed-side scan controller action is unavailable"
            )
            self._publish_scan_progress(message, active_bed_id=bed_id, error=True)
            return False, message

        goal = ExecuteBedSideScan.Goal()
        goal.bed_id = bed_id
        goal.side = side
        goal.start_endpoint = self._side_endpoint(
            bed_id,
            side,
            "start",
            goal_handle.request.target_pose,
        )
        goal.end_endpoint = self._side_endpoint(
            bed_id,
            side,
            "end",
            goal_handle.request.target_pose,
        )
        goal.min_flower_count = max(0, self._int_parameter("min_flowers_per_bed_side"))
        goal.harvest_enabled = self._harvest_enabled
        goal.dry_run = self._bool_parameter("bed_side_scan_dry_run")

        message = (
            f"Starting bed-side scan controller for {bed_id}:{side}; "
            f"dry_run={goal.dry_run}"
        )
        self._publish_scan_progress(
            message,
            active_bed_id=bed_id,
            active_scan_position_id=f"{bed_id}:{side}",
            detection_status="starting_bed_side_scan",
        )
        self._publish_feedback(goal_handle, message, 0.25)

        send_goal_future = self._bed_side_scan_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_bed_side_scan_feedback(
                goal_handle,
                feedback,
            ),
        )
        send_deadline = time.monotonic() + 5.0
        while rclpy.ok() and not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            if time.monotonic() >= send_deadline:
                message = (
                    f"{FailureCode.PRECONDITION_FAILED}: "
                    "timed out sending bed-side scan goal"
                )
                self._publish_scan_progress(message, active_bed_id=bed_id, error=True)
                return False, message
            time.sleep(0.05)

        bed_side_goal_handle = send_goal_future.result()
        if bed_side_goal_handle is None or not bed_side_goal_handle.accepted:
            message = f"{FailureCode.PLANNING_FAILED}: bed-side scan rejected"
            self._publish_scan_progress(message, active_bed_id=bed_id, error=True)
            return False, message

        self._active_bed_side_goal_handle = bed_side_goal_handle
        result_future = bed_side_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_active_work()
                return False, "INSPECT_BED cancelled"
            time.sleep(0.1)

        self._active_bed_side_goal_handle = None
        wrapped_result = result_future.result()
        if wrapped_result is None:
            message = f"{FailureCode.EXECUTION_FAILED}: bed-side scan returned no result"
            self._publish_scan_progress(message, active_bed_id=bed_id, error=True)
            return False, message

        result = wrapped_result.result
        if result.success:
            message = (
                f"INSPECT_BED completed for {bed_id}:{side}; "
                f"flowers_detected={result.flowers_detected}"
            )
            self._publish_feedback(goal_handle, message, 1.0)
            return True, message

        message = result.message or f"{FailureCode.EXECUTION_FAILED}: bed-side scan failed"
        self._publish_scan_progress(message, active_bed_id=bed_id, error=True)
        return False, message

    def _execute_inspect_flower(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.INSPECT_FLOWER)
        if not success:
            return False, message
        flower_id = goal_handle.request.target_id.strip()
        if not flower_id:
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "INSPECT_FLOWER requires target_id"
            )
            self._publish_scan_progress(message, error=True)
            return False, message
        message = (
            f"{FailureCode.NOT_IMPLEMENTED}: INSPECT_FLOWER accepted for "
            f"'{flower_id}', but single-position scan execution is not implemented yet."
        )
        self._publish_scan_progress(
            message,
            active_flower_id=flower_id,
            detection_status="waiting_for_scan_position",
            error=True,
        )
        self._publish_feedback(goal_handle, message, 1.0)
        # TODO: Resolve flower_id to a ScanPosition and run one scan attempt.
        return False, message

    def _execute_harvest(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.HARVEST)
        if not success:
            return False, message
        flower_id = goal_handle.request.target_id.strip()
        if not self._harvest_enabled:
            message = f"{FailureCode.HARVEST_DISABLED}: harvest_enabled is false"
            self._publish_harvest_status(
                "disabled",
                message,
                active_flower_id=flower_id,
                error=True,
            )
            return False, message
        if not flower_id:
            message = (
                f"{FailureCode.PRECONDITION_FAILED}: "
                "HARVEST requires target_id=<flower_id>"
            )
            self._publish_harvest_status("failed", message, error=True)
            return False, message
        message = (
            f"{FailureCode.NOT_IMPLEMENTED}: HARVEST accepted for '{flower_id}', "
            "but physical harvest execution is not implemented yet."
        )
        self._publish_harvest_status(
            "not_implemented",
            message,
            active_flower_id=flower_id,
            error=True,
        )
        self._publish_feedback(goal_handle, message, 1.0)
        # TODO: Add validated arm/gripper pose sequence before enabling physical
        # harvest, then add visual-servo alignment.
        return False, message

    def _on_nav2_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        progress = 0.5
        if hasattr(feedback, "distance_remaining"):
            distance = max(0.0, float(feedback.distance_remaining))
            progress = max(0.35, min(0.95, 1.0 / (1.0 + distance)))
        self._publish_feedback(goal_handle, "Nav2 driving", progress)
        self._publish_navigation_status("driving", "Nav2 driving")

    def _on_bed_side_scan_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        step = (
            f"Bed-side scan {feedback.phase}: "
            f"flowers={feedback.flowers_detected}, retries={feedback.retry_count}"
        )
        if feedback.message:
            step = feedback.message
        self._publish_feedback(goal_handle, step, feedback.progress)

    def _on_scan_position_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        step = (
            f"Scan pose {feedback.phase}: "
            f"distance_error={feedback.distance_error_m:.3f}m, "
            f"yaw_error={feedback.yaw_error_rad:.3f}rad"
        )
        if feedback.message:
            step = feedback.message
        self._publish_feedback(goal_handle, step, feedback.progress)

    def _on_plant_analysis_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        step = f"Plant analysis {feedback.phase}"
        if feedback.message:
            step = feedback.message
        self._publish_feedback(goal_handle, step, feedback.progress)

    def _cancel_active_work(self) -> None:
        self._publish_zero_twist()
        if self._active_nav_goal_handle is not None:
            try:
                self._active_nav_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Could not cancel Nav2 goal: {exc}")
            self._active_nav_goal_handle = None
        if self._active_bed_side_goal_handle is not None:
            try:
                self._active_bed_side_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Could not cancel bed-side scan goal: {exc}")
            self._active_bed_side_goal_handle = None
        if self._active_scan_position_goal_handle is not None:
            try:
                self._active_scan_position_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not cancel scan-position goal: {exc}"
                )
            self._active_scan_position_goal_handle = None
        if self._active_plant_analysis_goal_handle is not None:
            try:
                self._active_plant_analysis_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not cancel plant analysis goal: {exc}"
                )
            self._active_plant_analysis_goal_handle = None

    def _publish_zero_twist(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _publish_feedback(self, goal_handle, step: str, progress: float) -> None:
        feedback = ExecuteBehavior.Feedback()
        feedback.current_step = step
        feedback.progress = max(0.0, min(1.0, float(progress)))
        goal_handle.publish_feedback(feedback)

    def _publish_navigation_status(
        self,
        phase: str,
        message: str,
        target_pose: PoseStamped | None = None,
        error: bool = False,
    ) -> None:
        status = NavigationStatus()
        status.phase = phase
        status.current_pose = self._latest_pose
        status.target_pose = target_pose if target_pose is not None else PoseStamped()
        status.current_path = self._latest_path
        status.progress = 0.0 if error else 0.5
        status.replanning = phase == "replanning"
        status.obstacle_detected = False
        status.error = error
        status.message = message
        self._navigation_status_publisher.publish(status)

    def _publish_current_mission(
        self,
        mission_id: str,
        phase: str,
        message: str,
        queued_position: QueuedScanPosition | None = None,
        queue_index: int = 0,
        queue_total: int = 0,
        target_pose: PoseStamped | None = None,
        error: bool = False,
    ) -> None:
        status = CurrentMission()
        status.mission_id = mission_id
        status.phase = phase
        if queued_position is not None:
            status.active_bed_id = queued_position.scan_position.bed_id
            status.active_side = queued_position.side
            status.active_scan_position_id = (
                queued_position.scan_position.scan_position_id
            )
        status.queue_index = int(max(0, queue_index))
        status.queue_total = int(max(0, queue_total))
        status.current_pose = self._latest_pose
        status.target_pose = target_pose if target_pose is not None else PoseStamped()
        status.latest_plant_health = self._latest_plant_health
        status.error = bool(error)
        status.message = message
        self._current_mission_publisher.publish(status)

    def _publish_scan_progress(
        self,
        message: str,
        active_bed_id: str = "",
        active_scan_position_id: str = "",
        active_flower_id: str = "",
        detection_status: str = "",
        latest_plant_health: PlantHealth | None = None,
        error: bool = False,
    ) -> None:
        progress = ScanProgress()
        progress.active_bed_id = active_bed_id
        progress.active_scan_position_id = active_scan_position_id
        progress.active_flower_id = active_flower_id
        progress.detection_status = detection_status
        if latest_plant_health is not None:
            progress.latest_plant_health = latest_plant_health
        progress.error = error
        progress.message = message
        self._scan_progress_publisher.publish(progress)

    def _publish_harvest_status(
        self,
        phase: str,
        message: str,
        active_bed_id: str = "",
        active_flower_id: str = "",
        error: bool = False,
    ) -> None:
        status = HarvestStatus()
        status.active_bed_id = active_bed_id
        status.active_flower_id = active_flower_id
        status.phase = phase
        status.alignment_status = "not_started"
        status.harvest_enabled = self._harvest_enabled
        status.success = not error and phase in ("dry_run_ready", "configured")
        status.error = error
        status.message = message
        self._harvest_status_publisher.publish(status)

    def _missing_topics(self, health: TopicHealth, keys: list[str]) -> list[str]:
        missing = []
        for key in keys:
            if key == "scan" and not health.scan_seen:
                missing.append(self._string_parameter("scan_topic"))
            elif key == "odom" and not health.odom_seen:
                missing.append(self._string_parameter("odom_topic"))
            elif key == "map" and not health.map_seen:
                missing.append(self._string_parameter("map_topic"))
            elif key == "amcl" and not health.amcl_seen:
                missing.append(self._string_parameter("amcl_pose_topic"))
            elif key == "plan" and not health.plan_seen:
                missing.append(self._string_parameter("nav_plan_topic"))
        return missing

    def _pose_stamped(self, pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        return msg

    def _pose_is_finite(self, pose) -> bool:
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        return all(math.isfinite(float(value)) for value in values)

    def _parse_bed_side_target(self, target_id: str) -> tuple[str, str] | None:
        parts = [part.strip() for part in target_id.split(":")]
        if len(parts) != 2:
            return None
        bed_id, side = parts[0], parts[1].lower()
        if not bed_id or side not in ("a", "b"):
            return None
        return bed_id, side

    def _side_endpoint(
        self,
        bed_id: str,
        side: str,
        endpoint_name: str,
        pose,
    ) -> ScanPosition:
        endpoint = ScanPosition()
        endpoint.scan_position_id = f"{bed_id}:{side}:{endpoint_name}"
        endpoint.bed_id = bed_id
        endpoint.base_pose.x = float(pose.position.x)
        endpoint.base_pose.y = float(pose.position.y)
        endpoint.base_pose.theta = self._yaw_from_pose(pose)
        endpoint.order = 0 if endpoint_name == "start" else 1
        endpoint.enabled = True
        return endpoint

    def _yaw_from_pose(self, pose) -> float:
        x = float(pose.orientation.x)
        y = float(pose.orientation.y)
        z = float(pose.orientation.z)
        w = float(pose.orientation.w)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _set_mode_for_behavior(self, behavior: int) -> tuple[bool, str]:
        mode_by_behavior = {
            BehaviorType.IDLE: "AUTONOMOUS_IDLE",
            BehaviorType.TELEOP: "TELEOP",
            BehaviorType.MAP: "MAPPING",
            BehaviorType.LOCALIZE: "LOCALIZING",
            BehaviorType.NAVIGATE: "NAVIGATING",
            BehaviorType.INSPECT_BED: "SCANNING",
            BehaviorType.INSPECT_FLOWER: "SCANNING",
            BehaviorType.HARVEST: "HARVESTING",
            BehaviorType.ARM_TEST: "ARM_TEST",
        }
        mode = mode_by_behavior.get(behavior)
        if mode is None:
            return False, f"Unknown behavior type {behavior}"
        return self._state_machine.set_mode(mode)

    def _behavior_names(self) -> dict[int, str]:
        return {
            BehaviorType.IDLE: "IDLE",
            BehaviorType.TELEOP: "TELEOP",
            BehaviorType.MAP: "MAP",
            BehaviorType.LOCALIZE: "LOCALIZE",
            BehaviorType.INSPECT_BED: "INSPECT_BED",
            BehaviorType.INSPECT_FLOWER: "INSPECT_FLOWER",
            BehaviorType.HARVEST: "HARVEST",
            BehaviorType.ARM_TEST: "ARM_TEST",
            BehaviorType.NAVIGATE: "NAVIGATE",
        }

    def _behavior_name(self, behavior: int) -> str:
        return self._behavior_names().get(behavior, f"UNKNOWN_{behavior}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

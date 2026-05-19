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
from simbiosys_interfaces.action import ExecuteBehavior
from simbiosys_interfaces.msg import (
    BehaviorType,
    HarvestStatus,
    NavigationStatus,
    ScanProgress,
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

        self.declare_parameter("harvest_enabled", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/mirte_base_controller/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("nav_plan_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/mirte_base_controller/cmd_vel")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("nav2_server_timeout_sec", 2.0)
        self.declare_parameter("require_localization_for_navigation", True)

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
        self._harvest_status_publisher = self.create_publisher(
            HarvestStatus,
            "simbiosys/harvest_status",
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
            lambda _msg: self._mark_seen("odom"),
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
            lambda _msg: self._mark_seen("amcl"),
            10,
        )
        self.create_subscription(
            Path,
            self._string_parameter("nav_plan_topic"),
            lambda _msg: self._mark_seen("plan"),
            10,
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

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
            message = "PRECONDITION_FAILED: target_pose contains non-finite values"
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        health = self._health_snapshot()
        required = ["odom", "map"]
        if self._bool_parameter("require_localization_for_navigation"):
            required.append("amcl")
        missing = self._missing_topics(health, required)
        if missing:
            message = "PRECONDITION_FAILED: navigation waiting for " + ", ".join(missing)
            self._publish_feedback(goal_handle, message, 0.1)
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        if self._nav2_client is None:
            message = "PRECONDITION_FAILED: nav2_msgs NavigateToPose is not available"
            self._publish_navigation_status("failed", message, error=True)
            return False, message

        self._publish_feedback(goal_handle, "Waiting for Nav2 NavigateToPose", 0.2)
        if not self._nav2_client.wait_for_server(
            timeout_sec=self._double_parameter("nav2_server_timeout_sec")
        ):
            message = "PRECONDITION_FAILED: Nav2 NavigateToPose action is unavailable"
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
                message = "PRECONDITION_FAILED: timed out sending Nav2 goal"
                self._publish_navigation_status("failed", message, goal.pose, error=True)
                return False, message
            time.sleep(0.05)
        nav_goal_handle = send_goal_future.result()
        if nav_goal_handle is None or not nav_goal_handle.accepted:
            message = "PLANNING_FAILED: Nav2 rejected the goal"
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
            message = "EXECUTION_FAILED: Nav2 returned no result"
            self._publish_navigation_status("failed", message, goal.pose, error=True)
            return False, message

        status = int(result.status)
        if status == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            message = "NAVIGATE completed"
            self._publish_feedback(goal_handle, message, 1.0)
            self._publish_navigation_status("arrived", message, goal.pose)
            self._set_mode_for_behavior(BehaviorType.IDLE)
            return True, message

        message = f"EXECUTION_FAILED: Nav2 finished with status {status}"
        self._publish_navigation_status("failed", message, goal.pose, error=True)
        return False, message

    def _execute_inspect_bed(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.INSPECT_BED)
        if not success:
            return False, message
        bed_id = goal_handle.request.target_id.strip()
        if not bed_id:
            message = "PRECONDITION_FAILED: INSPECT_BED requires target_id=<bed_id>"
            self._publish_scan_progress(message, error=True)
            return False, message
        message = (
            f"INSPECT_BED accepted for bed '{bed_id}'. "
            "Scan-position execution is ready for metadata integration."
        )
        self._publish_scan_progress(
            message,
            active_bed_id=bed_id,
            detection_status="waiting_for_scan_positions",
        )
        self._publish_feedback(goal_handle, message, 1.0)
        return True, message

    def _execute_inspect_flower(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.INSPECT_FLOWER)
        if not success:
            return False, message
        flower_id = goal_handle.request.target_id.strip()
        if not flower_id:
            message = "PRECONDITION_FAILED: INSPECT_FLOWER requires target_id"
            self._publish_scan_progress(message, error=True)
            return False, message
        message = (
            f"INSPECT_FLOWER accepted for '{flower_id}'. "
            "Single-position scan execution is ready for metadata integration."
        )
        self._publish_scan_progress(
            message,
            active_flower_id=flower_id,
            detection_status="waiting_for_scan_position",
        )
        self._publish_feedback(goal_handle, message, 1.0)
        return True, message

    def _execute_harvest(self, goal_handle) -> tuple[bool, str]:
        success, message = self._set_mode_for_behavior(BehaviorType.HARVEST)
        if not success:
            return False, message
        flower_id = goal_handle.request.target_id.strip()
        if not self._harvest_enabled:
            message = "HARVEST_DISABLED: harvest_enabled is false"
            self._publish_harvest_status(
                "disabled",
                message,
                active_flower_id=flower_id,
                error=True,
            )
            return False, message
        if not flower_id:
            message = "PRECONDITION_FAILED: HARVEST requires target_id=<flower_id>"
            self._publish_harvest_status("failed", message, error=True)
            return False, message
        message = (
            f"HARVEST accepted for '{flower_id}'. "
            "Physical harvest sequence is gated until arm/gripper poses are validated."
        )
        self._publish_harvest_status(
            "dry_run_ready",
            message,
            active_flower_id=flower_id,
        )
        self._publish_feedback(goal_handle, message, 1.0)
        return True, message

    def _on_nav2_feedback(self, goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        progress = 0.5
        if hasattr(feedback, "distance_remaining"):
            distance = max(0.0, float(feedback.distance_remaining))
            progress = max(0.35, min(0.95, 1.0 / (1.0 + distance)))
        self._publish_feedback(goal_handle, "Nav2 driving", progress)
        self._publish_navigation_status("driving", "Nav2 driving")

    def _cancel_active_work(self) -> None:
        self._publish_zero_twist()
        if self._active_nav_goal_handle is not None:
            try:
                self._active_nav_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Could not cancel Nav2 goal: {exc}")
            self._active_nav_goal_handle = None

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
        status.target_pose = target_pose if target_pose is not None else PoseStamped()
        status.current_path = Path()
        status.progress = 0.0 if error else 0.5
        status.replanning = phase == "replanning"
        status.obstacle_detected = False
        status.error = error
        status.message = message
        self._navigation_status_publisher.publish(status)

    def _publish_scan_progress(
        self,
        message: str,
        active_bed_id: str = "",
        active_scan_position_id: str = "",
        active_flower_id: str = "",
        detection_status: str = "",
        error: bool = False,
    ) -> None:
        progress = ScanProgress()
        progress.active_bed_id = active_bed_id
        progress.active_scan_position_id = active_scan_position_id
        progress.active_flower_id = active_flower_id
        progress.detection_status = detection_status
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

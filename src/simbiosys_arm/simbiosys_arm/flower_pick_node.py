import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from simbiosys_interfaces.msg import FlowerTarget
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


MOVEIT_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
}

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_joint",
]

JOINT_ONLY_POSES = {"stow", "storage"}
JOINT_LIMIT = math.pi / 2.0

MIRTE_CHAIN = [
    {
        "origin_xyz": (0.0, 0.0, 0.10),
        "origin_rpy": (0.0, 0.0, math.pi / 2.0),
        "joint_name": None,
        "axis": None,
    },
    {
        "origin_xyz": (0.0, -0.079274, 0.06),
        "origin_rpy": (math.pi / 2.0, 0.0, math.pi),
        "joint_name": "shoulder_pan_joint",
        "axis": (0.0, 1.0, 0.0),
    },
    {
        "origin_xyz": (0.0, 0.0281, -0.00625),
        "origin_rpy": (math.pi, 0.0, math.pi / 2.0),
        "joint_name": "shoulder_lift_joint",
        "axis": (0.0, 1.0, 0.0),
    },
    {
        "origin_xyz": (0.1378, 0.0, 0.0),
        "origin_rpy": (-math.pi, 0.0, math.pi / 2.0),
        "joint_name": "elbow_joint",
        "axis": (1.0, 0.0, 0.0),
    },
    {
        "origin_xyz": (-0.00014, 0.14265, 0.0),
        "origin_rpy": (3.1416, 0.0, 1.5708),
        "joint_name": "wrist_joint",
        "axis": (0.0, 1.0, 0.0),
    },
]


class FlowerPickNode(Node):
    """Simple callable flower picking routine for MIRTE + MoveIt."""

    def __init__(self) -> None:
        super().__init__("flower_pick_node")
        self._callback_group = ReentrantCallbackGroup()
        self._target_lock = threading.Lock()
        self._latest_target: FlowerTarget | None = None
        self._active_target: FlowerTarget | None = None

        self.declare_parameter("flower_target_topic", "simbiosys/flower_target")
        self.declare_parameter("use_fake_target_if_missing", True)
        self.declare_parameter("require_ready_for_harvest", False)
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("image_width_px", 640.0)
        self.declare_parameter("image_height_px", 480.0)
        self.declare_parameter("fake_bbox_center_x_px", 320.0)
        self.declare_parameter("fake_bbox_center_y_px", 240.0)
        self.declare_parameter("bbox_center_tolerance_px", 60.0)

        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("motion_backend", "simple_ik")
        self.declare_parameter("fallback_to_simple_ik_on_moveit_failure", True)
        self.declare_parameter("enable_arm_service", "/enable_arm_control")
        self.declare_parameter(
            "arm_follow_joint_trajectory_action",
            "/mirte_master_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "arm_trajectory_topic",
            "/mirte_master_arm_controller/joint_trajectory",
        )
        self.declare_parameter("joint_motion_duration_sec", 2.5)
        self.declare_parameter("wait_for_arm_action_result", False)
        self.declare_parameter("planning_group", "mirte_arm")
        self.declare_parameter("target_link", "wrist")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("execute", True)
        self.declare_parameter("planning_time_sec", 5.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("position_tolerance_m", 0.025)
        self.declare_parameter("use_orientation_constraint", False)
        self.declare_parameter("orientation_tolerance_rad", 0.35)
        self.declare_parameter("velocity_scaling", 0.12)
        self.declare_parameter("acceleration_scaling", 0.12)
        self.declare_parameter("workspace_size", 1.0)
        self.declare_parameter("server_timeout_sec", 3.0)
        self.declare_parameter("result_timeout_sec", 35.0)
        self.declare_parameter("simple_ik_tolerance_m", 0.035)
        self.declare_parameter("simple_ik_shoulder_pan_joint", 0.0)
        self.declare_parameter("horizontal_wrist_sum_rad", -math.pi / 2.0)
        self.declare_parameter("simple_ik_use_bbox_offsets", True)
        self.declare_parameter("bbox_x_to_pan_gain_rad_per_px", 0.001)
        self.declare_parameter("max_bbox_pan_offset_rad", 0.35)
        self.declare_parameter("bbox_y_to_z_gain_m_per_px", 0.0005)
        self.declare_parameter("max_bbox_z_offset_m", 0.08)
        self.declare_parameter("flower_distance_m", 0.28)
        self.declare_parameter("fallback_flower_height_m", 0.20)
        self.declare_parameter("inspect_distance_offset_m", 0.06)
        self.declare_parameter("pre_grasp_distance_offset_m", 0.03)
        self.declare_parameter("grasp_below_head_m", 0.015)
        self.declare_parameter("lift_above_grasp_m", 0.16)
        self.declare_parameter("ready_above_head_m", 0.12)
        self.declare_parameter("ready_distance_m", 0.23)

        self.declare_parameter("horizontal_qx", 0.0)
        self.declare_parameter("horizontal_qy", 0.7071)
        self.declare_parameter("horizontal_qz", 0.0)
        self.declare_parameter("horizontal_qw", 0.7071)

        self.declare_parameter("inspect_x", 0.31)
        self.declare_parameter("inspect_y", 0.0)
        self.declare_parameter("inspect_z", 0.20)
        self.declare_parameter("ready_x", 0.23)
        self.declare_parameter("ready_y", 0.0)
        self.declare_parameter("ready_z", 0.32)
        self.declare_parameter("pre_grasp_x", 0.28)
        self.declare_parameter("pre_grasp_y", 0.0)
        self.declare_parameter("pre_grasp_z", 0.11)
        self.declare_parameter("grasp_x", 0.25)
        self.declare_parameter("grasp_y", 0.0)
        self.declare_parameter("grasp_z", 0.08)
        self.declare_parameter("lift_x", 0.30)
        self.declare_parameter("lift_y", 0.0)
        self.declare_parameter("lift_z", 0.24)
        self.declare_parameter("storage_x", 0.08)
        self.declare_parameter("storage_y", -0.12)
        self.declare_parameter("storage_z", 0.42)
        self.declare_parameter("inspect_joints", [0.0, -1.0, -1.35, 1.25])
        self.declare_parameter("pre_grasp_joints", [0.0, -1.2, -1.45, 1.35])
        self.declare_parameter("grasp_joints", [0.0, -1.35, -1.55, 1.45])
        self.declare_parameter("lift_joints", [0.0, -0.65, -1.35, 0.9])
        self.declare_parameter("storage_joints", [math.pi / 2.0, -0.25, -1.2, -0.1207963267948966])
        self.declare_parameter("stow_joints", [0.0, 1.57079632, -1.5707963267948966, -1.57079632])

        self.declare_parameter(
            "gripper_action",
            "/mirte_master_gripper_controller/gripper_cmd",
        )
        self.declare_parameter("open_position", -0.75)
        self.declare_parameter("close_position", 0.2)
        self.declare_parameter("max_effort", 0.0)

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self._string_parameter("arm_trajectory_topic"),
            10,
        )
        self.create_subscription(
            FlowerTarget,
            self._string_parameter("flower_target_topic"),
            self._on_flower_target,
            10,
            callback_group=self._callback_group,
        )
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            self._string_parameter("move_group_action"),
            callback_group=self._callback_group,
        )
        self._arm_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._string_parameter("arm_follow_joint_trajectory_action"),
            callback_group=self._callback_group,
        )
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            self._string_parameter("gripper_action"),
            callback_group=self._callback_group,
        )
        self._enable_arm_client = self.create_client(
            SetBool,
            self._string_parameter("enable_arm_service"),
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "simbiosys/pick_flower",
            self._on_pick_flower,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Flower pick routine ready. Call /simbiosys/pick_flower to run it."
        )

    def _on_flower_target(self, msg: FlowerTarget) -> None:
        with self._target_lock:
            self._latest_target = msg

    def _on_pick_flower(self, _request, response):
        success, message = self._run_pick_sequence()
        response.success = success
        response.message = message
        return response

    def _run_pick_sequence(self) -> tuple[bool, str]:
        target = self._target_or_fake()
        valid, message = self._validate_target(target)
        if not valid:
            return False, message

        self.get_logger().info(
            f"Starting flower pick for {target.flower_id or 'unnamed flower'}"
        )
        steps = [
            ("move_stow_start", lambda: self._move_to_pose("stow")),
            ("open_gripper", lambda: self._send_gripper(self._double_parameter("open_position"))),
            ("move_ready_above", lambda: self._move_to_pose("ready")),
            ("move_inspect", lambda: self._move_to_pose("inspect")),
            ("move_pre_grasp", lambda: self._move_to_pose("pre_grasp")),
            ("move_grasp", lambda: self._move_to_pose("grasp")),
            ("close_gripper", lambda: self._send_gripper(self._double_parameter("close_position"))),
            ("lift", lambda: self._move_to_pose("lift")),
            ("move_storage", lambda: self._move_to_pose("storage")),
            ("open_gripper_drop", lambda: self._send_gripper(self._double_parameter("open_position"))),
            ("move_stow_end", lambda: self._move_to_pose("stow")),
        ]

        self._active_target = target
        try:
            for step_name, step in steps:
                self.get_logger().info(f"Pick step: {step_name}")
                ok, step_message = step()
                if not ok:
                    message = f"Pick failed during {step_name}: {step_message}"
                    self.get_logger().warning(message)
                    return False, message
                time.sleep(0.5)

            message = "Flower pick sequence completed"
            self.get_logger().info(message)
            return True, message
        finally:
            self._active_target = None

    def _target_or_fake(self) -> FlowerTarget:
        with self._target_lock:
            target = self._latest_target
        if target is not None:
            return target

        fake = FlowerTarget()
        fake.flower_id = "fake_target"
        fake.detected = bool(self._bool_parameter("use_fake_target_if_missing"))
        fake.ready_for_harvest = True
        fake.confidence = 1.0
        fake.bbox_center_px.x = self._double_parameter("fake_bbox_center_x_px")
        fake.bbox_center_px.y = self._double_parameter("fake_bbox_center_y_px")
        fake.height_cm = self._double_parameter("fallback_flower_height_m") * 100.0
        fake.message = "Generated from flower_pick_node fake target parameters"
        return fake

    def _validate_target(self, target: FlowerTarget) -> tuple[bool, str]:
        if not target.detected:
            return False, "No FlowerTarget received and fake target fallback is disabled"
        if (
            self._bool_parameter("require_ready_for_harvest")
            and not target.ready_for_harvest
        ):
            return False, f"Flower '{target.flower_id}' is not ready for harvest"
        if target.confidence < self._double_parameter("min_confidence"):
            return (
                False,
                f"Flower confidence {target.confidence:.2f} is below min_confidence",
            )
        return True, "target accepted"

    def _send_gripper(self, position: float) -> tuple[bool, str]:
        if not self._gripper_client.wait_for_server(
            timeout_sec=self._double_parameter("server_timeout_sec")
        ):
            return False, (
                f"Gripper action {self._string_parameter('gripper_action')} "
                "is not available"
            )

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self._double_parameter("max_effort")

        goal_handle, error = self._send_action_goal(self._gripper_client, goal)
        if error:
            return False, error
        if goal_handle is None or not goal_handle.accepted:
            return False, "gripper rejected goal"

        result, error = self._wait_for_action_result(goal_handle, timeout_sec=8.0)
        if error:
            return False, error
        if result is None:
            return False, "gripper returned no result"
        return True, f"gripper moved to {position:.3f}"

    def _move_to_pose(self, pose_name: str) -> tuple[bool, str]:
        if pose_name in JOINT_ONLY_POSES:
            return self._move_to_joint_pose_action(pose_name)

        backend = self._string_parameter("motion_backend").strip().lower()
        if backend == "direct_joints":
            return self._move_to_joint_pose(pose_name)
        if backend == "direct_action":
            return self._move_to_joint_pose_action(pose_name)
        if backend == "simple_ik":
            return self._move_to_simple_ik_pose(pose_name)
        if backend != "moveit":
            return False, (
                f"Unknown motion_backend '{backend}'. "
                "Use 'direct_action', 'direct_joints', 'simple_ik', or 'moveit'."
            )
        if not self._has_task_space_pose(pose_name):
            return False, f"No MoveIt task-space parameters declared for pose '{pose_name}'"

        if not self._move_group_client.wait_for_server(
            timeout_sec=self._double_parameter("server_timeout_sec")
        ):
            return False, (
                f"MoveIt action {self._string_parameter('move_group_action')} "
                "is not available"
            )

        pose = self._pose_from_parameters(pose_name)
        goal = MoveGroup.Goal()
        goal.request = self._motion_request(pose_name, pose)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = not self._bool_parameter("execute")
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.1

        goal_handle, error = self._send_action_goal(self._move_group_client, goal)
        if error:
            return False, error
        if goal_handle is None or not goal_handle.accepted:
            return False, f"MoveIt rejected {pose_name} goal"

        wrapped_result, error = self._wait_for_action_result(
            goal_handle,
            timeout_sec=self._double_parameter("result_timeout_sec"),
        )
        if error:
            return False, error
        if wrapped_result is None:
            return False, f"MoveIt returned no result for {pose_name}"

        result = wrapped_result.result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            code_name = MOVEIT_ERROR_NAMES.get(code, f"UNKNOWN_{code}")
            if self._bool_parameter("fallback_to_simple_ik_on_moveit_failure"):
                self.get_logger().warning(
                    f"MoveIt failed {pose_name}: {code_name} ({code}); "
                    "falling back to simple_ik"
                )
                return self._move_to_simple_ik_pose(pose_name)
            return False, f"MoveIt failed {pose_name}: {code_name} ({code})"
        action = "executed" if self._bool_parameter("execute") else "planned"
        return True, f"{action} {pose_name}"

    def _move_to_joint_pose(self, pose_name: str) -> tuple[bool, str]:
        enabled, message = self._ensure_arm_enabled()
        if not enabled:
            return False, message

        positions = list(
            self.get_parameter(f"{pose_name}_joints")
            .get_parameter_value()
            .double_array_value
        )
        if len(positions) != len(ARM_JOINT_NAMES):
            return (
                False,
                f"{pose_name}_joints must contain {len(ARM_JOINT_NAMES)} values",
            )
        positions = self._clamp_joint_positions(positions)

        subscription_count = self._wait_for_arm_trajectory_subscriber(timeout_sec=1.0)
        if subscription_count == 0:
            return (
                False,
                f"No subscribers on {self._string_parameter('arm_trajectory_topic')}",
            )

        duration = max(0.1, self._double_parameter("joint_motion_duration_sec"))
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points = [point]

        self._trajectory_publisher.publish(trajectory)
        time.sleep(duration + 0.2)
        return True, f"published direct joint pose {pose_name}"

    def _move_to_joint_pose_action(self, pose_name: str) -> tuple[bool, str]:
        enabled, message = self._ensure_arm_enabled()
        if not enabled:
            return False, message

        positions = list(
            self.get_parameter(f"{pose_name}_joints")
            .get_parameter_value()
            .double_array_value
        )
        if len(positions) != len(ARM_JOINT_NAMES):
            return (
                False,
                f"{pose_name}_joints must contain {len(ARM_JOINT_NAMES)} values",
            )
        positions = self._clamp_joint_positions(positions)
        if not self._arm_trajectory_client.wait_for_server(
            timeout_sec=self._double_parameter("server_timeout_sec")
        ):
            return False, (
                "Arm FollowJointTrajectory action "
                f"{self._string_parameter('arm_follow_joint_trajectory_action')} "
                "is not available"
            )

        duration = max(0.1, self._double_parameter("joint_motion_duration_sec"))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = self._clamp_joint_positions(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]

        goal_handle, error = self._send_action_goal(self._arm_trajectory_client, goal)
        if error:
            return False, error
        if goal_handle is None or not goal_handle.accepted:
            return False, f"arm controller rejected {pose_name} goal"

        if not self._bool_parameter("wait_for_arm_action_result"):
            time.sleep(duration + 0.3)
            return True, f"sent joint action pose {pose_name}"

        wrapped_result, error = self._wait_for_action_result(
            goal_handle,
            timeout_sec=duration + 5.0,
        )
        if error:
            return False, error
        if wrapped_result is None:
            return False, f"arm controller returned no result for {pose_name}"
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, (
                f"arm controller failed {pose_name}: "
                f"error_code={result.error_code}, error='{result.error_string}'"
            )
        return True, f"executed joint action pose {pose_name}"

    def _move_to_simple_ik_pose(self, pose_name: str) -> tuple[bool, str]:
        enabled, message = self._ensure_arm_enabled()
        if not enabled:
            return False, message

        if not self._has_task_space_pose(pose_name):
            return False, f"No task-space parameters declared for pose '{pose_name}'"

        target, shoulder_pan = self._simple_ik_target_for_pose(pose_name)
        self.get_logger().info(
            f"simple_ik {pose_name}: target=({target[0]:.3f}, "
            f"{target[1]:.3f}, {target[2]:.3f}), pan={shoulder_pan:.3f}"
        )
        solution, error = self._solve_simple_ik(target, shoulder_pan)
        if solution is None:
            return (
                False,
                f"simple_ik could not reach {pose_name}; closest error={error:.3f}m",
            )

        return self._send_joint_positions(
            pose_name,
            [
                solution["shoulder_pan_joint"],
                solution["shoulder_lift_joint"],
                solution["elbow_joint"],
                solution["wrist_joint"],
            ],
        )

    def _solve_simple_ik(
        self,
        target: tuple[float, float, float],
        shoulder_pan: float,
    ) -> tuple[dict[str, float] | None, float]:
        joints = {
            "shoulder_pan_joint": shoulder_pan,
            "shoulder_lift_joint": -1.2,
            "elbow_joint": -1.45,
            "wrist_joint": 0.0,
        }
        joints["wrist_joint"] = self._leveled_wrist_joint(
            joints["shoulder_lift_joint"],
            joints["elbow_joint"],
        )
        best = joints.copy()
        best_error = self._wrist_position_error(best, target)
        step = 0.35
        while step > 0.002:
            improved = False
            for joint_name in ("shoulder_lift_joint", "elbow_joint"):
                original = joints[joint_name]
                local_best = original
                for direction in (1.0, -1.0):
                    joints[joint_name] = self._clamp_joint(original + direction * step)
                    joints["wrist_joint"] = self._leveled_wrist_joint(
                        joints["shoulder_lift_joint"],
                        joints["elbow_joint"],
                    )
                    error = self._wrist_position_error(joints, target)
                    if error < best_error:
                        best_error = error
                        best = joints.copy()
                        local_best = joints[joint_name]
                        improved = True
                joints[joint_name] = local_best
                joints["wrist_joint"] = self._leveled_wrist_joint(
                    joints["shoulder_lift_joint"],
                    joints["elbow_joint"],
                )
            if best_error <= self._double_parameter("simple_ik_tolerance_m"):
                break
            if not improved:
                step *= 0.5

        if best_error > self._double_parameter("simple_ik_tolerance_m"):
            return None, best_error
        best["wrist_joint"] = self._leveled_wrist_joint(
            best["shoulder_lift_joint"],
            best["elbow_joint"],
        )
        return best, best_error

    def _leveled_wrist_joint(self, shoulder_lift: float, elbow: float) -> float:
        return self._clamp_joint(
            self._double_parameter("horizontal_wrist_sum_rad") - shoulder_lift - elbow
        )

    def _simple_ik_target_for_pose(
        self,
        pose_name: str,
    ) -> tuple[tuple[float, float, float], float]:
        x, y, z = self._base_target_for_pose(pose_name)
        shoulder_pan = self._double_parameter("simple_ik_shoulder_pan_joint")

        target = self._active_target
        if target is None or not self._bool_parameter("simple_ik_use_bbox_offsets"):
            return (x, y, z), shoulder_pan

        x_error_px = float(target.bbox_center_px.x) - (
            self._double_parameter("image_width_px") / 2.0
        )
        y_error_px = float(target.bbox_center_px.y) - (
            self._double_parameter("image_height_px") / 2.0
        )

        pan_offset = self._clamp(
            x_error_px * self._double_parameter("bbox_x_to_pan_gain_rad_per_px"),
            -self._double_parameter("max_bbox_pan_offset_rad"),
            self._double_parameter("max_bbox_pan_offset_rad"),
        )
        z_offset = self._clamp(
            -y_error_px * self._double_parameter("bbox_y_to_z_gain_m_per_px"),
            -self._double_parameter("max_bbox_z_offset_m"),
            self._double_parameter("max_bbox_z_offset_m"),
        )
        return (x, y, z + z_offset), self._clamp_joint(shoulder_pan + pan_offset)

    def _base_target_for_pose(self, pose_name: str) -> tuple[float, float, float]:
        target = self._active_target
        if target is None:
            return (
                self._double_parameter(f"{pose_name}_x"),
                self._double_parameter(f"{pose_name}_y"),
                self._double_parameter(f"{pose_name}_z"),
            )

        flower_x = self._double_parameter("flower_distance_m")
        flower_y = 0.0
        flower_z = self._flower_head_height_m(target)
        grasp_z = max(0.02, flower_z - self._double_parameter("grasp_below_head_m"))

        if pose_name == "ready":
            return (
                self._double_parameter("ready_distance_m"),
                flower_y,
                flower_z + self._double_parameter("ready_above_head_m"),
            )
        if pose_name == "inspect":
            return (
                flower_x + self._double_parameter("inspect_distance_offset_m"),
                flower_y,
                flower_z,
            )
        if pose_name == "pre_grasp":
            return (
                flower_x + self._double_parameter("pre_grasp_distance_offset_m"),
                flower_y,
                grasp_z,
            )
        if pose_name == "grasp":
            return (flower_x, flower_y, grasp_z)
        if pose_name == "lift":
            return (
                flower_x,
                flower_y,
                grasp_z + self._double_parameter("lift_above_grasp_m"),
            )
        return (
            self._double_parameter(f"{pose_name}_x"),
            self._double_parameter(f"{pose_name}_y"),
            self._double_parameter(f"{pose_name}_z"),
        )

    def _flower_head_height_m(self, target: FlowerTarget) -> float:
        if target.height_cm > 0.0 and math.isfinite(float(target.height_cm)):
            return float(target.height_cm) / 100.0
        return self._double_parameter("fallback_flower_height_m")

    def _send_joint_positions(
        self,
        pose_name: str,
        positions: list[float],
    ) -> tuple[bool, str]:
        if not self._arm_trajectory_client.wait_for_server(
            timeout_sec=self._double_parameter("server_timeout_sec")
        ):
            return False, (
                "Arm FollowJointTrajectory action "
                f"{self._string_parameter('arm_follow_joint_trajectory_action')} "
                "is not available"
            )

        duration = max(0.1, self._double_parameter("joint_motion_duration_sec"))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]

        goal_handle, error = self._send_action_goal(self._arm_trajectory_client, goal)
        if error:
            return False, error
        if goal_handle is None or not goal_handle.accepted:
            return False, f"arm controller rejected {pose_name} goal"

        if not self._bool_parameter("wait_for_arm_action_result"):
            time.sleep(duration + 0.3)
            return True, f"sent joint pose {pose_name}"

        wrapped_result, error = self._wait_for_action_result(
            goal_handle,
            timeout_sec=duration + 5.0,
        )
        if error:
            return False, error
        if wrapped_result is None:
            return False, f"arm controller returned no result for {pose_name}"
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, (
                f"arm controller failed {pose_name}: "
                f"error_code={result.error_code}, error='{result.error_string}'"
            )
        return True, f"executed joint pose {pose_name}"

    def _has_task_space_pose(self, pose_name: str) -> bool:
        return all(
            self.has_parameter(f"{pose_name}_{axis}")
            for axis in ("x", "y", "z")
        )

    def _ensure_arm_enabled(self) -> tuple[bool, str]:
        if not self._enable_arm_client.wait_for_service(timeout_sec=0.5):
            return True, "arm enable service unavailable; continuing"

        request = SetBool.Request()
        request.data = True
        event = threading.Event()
        state = {"result": None, "error": None}
        future = self._enable_arm_client.call_async(request)
        future.add_done_callback(lambda done: self._store_result(done, event, state))
        if not event.wait(1.0):
            return False, "timed out calling /enable_arm_control"
        if state["error"]:
            return False, f"failed to enable arm control: {state['error']}"
        result = state["result"]
        if result is None:
            return False, "enable arm control returned no result"
        if not result.success:
            return False, result.message or "enable arm control rejected request"
        return True, result.message or "arm control enabled"

    def _wait_for_arm_trajectory_subscriber(self, timeout_sec: float) -> int:
        deadline = time.monotonic() + timeout_sec
        count = self._trajectory_publisher.get_subscription_count()
        while count == 0 and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.05)
            count = self._trajectory_publisher.get_subscription_count()
        return count

    def _wrist_position_error(
        self,
        joint_positions: dict[str, float],
        target: tuple[float, float, float],
    ) -> float:
        position = self._wrist_position(joint_positions)
        dx = position[0] - target[0]
        dy = position[1] - target[1]
        dz = position[2] - target[2]
        return math.sqrt(dx * dx + dz * dz)

    def _wrist_position(
        self,
        joint_positions: dict[str, float],
    ) -> tuple[float, float, float]:
        transform = self._identity_transform()
        for joint in MIRTE_CHAIN:
            transform = self._matmul(
                transform,
                self._origin_transform(joint["origin_xyz"], joint["origin_rpy"]),
            )
            joint_name = joint["joint_name"]
            if joint_name is not None:
                transform = self._matmul(
                    transform,
                    self._axis_angle_transform(
                        joint["axis"],
                        joint_positions[joint_name],
                    ),
                )
        return (transform[0][3], transform[1][3], transform[2][3])

    @staticmethod
    def _identity_transform() -> list[list[float]]:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _origin_transform(
        self,
        xyz: tuple[float, float, float],
        rpy: tuple[float, float, float],
    ) -> list[list[float]]:
        rotation = self._rpy_matrix(*rpy)
        return [
            [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
            [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
            [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _axis_angle_transform(
        axis: tuple[float, float, float],
        angle: float,
    ) -> list[list[float]]:
        x, y, z = axis
        length = math.sqrt(x * x + y * y + z * z)
        x /= length
        y /= length
        z /= length
        c = math.cos(angle)
        s = math.sin(angle)
        one_c = 1.0 - c
        return [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, 0.0],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s, 0.0],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rpy_matrix(
        roll: float,
        pitch: float,
        yaw: float,
    ) -> list[list[float]]:
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]

    @staticmethod
    def _matmul(
        left: list[list[float]],
        right: list[list[float]],
    ) -> list[list[float]]:
        return [
            [
                sum(left[row][idx] * right[idx][col] for idx in range(4))
                for col in range(4)
            ]
            for row in range(4)
        ]

    @staticmethod
    def _clamp_joint(value: float) -> float:
        return min(JOINT_LIMIT, max(-JOINT_LIMIT, value))

    def _clamp_joint_positions(self, positions: list[float]) -> list[float]:
        return [self._clamp_joint(position) for position in positions]

    def _motion_request(self, pose_name: str, pose: Pose) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self._string_parameter("planning_group")
        request.num_planning_attempts = self._int_parameter("planning_attempts")
        request.allowed_planning_time = self._double_parameter("planning_time_sec")
        request.max_velocity_scaling_factor = self._double_parameter("velocity_scaling")
        request.max_acceleration_scaling_factor = self._double_parameter(
            "acceleration_scaling"
        )
        request.workspace_parameters = self._workspace_parameters()
        request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = f"{pose_name}_target"
        constraints.position_constraints = [self._position_constraint(pose)]
        if self._bool_parameter("use_orientation_constraint"):
            constraints.orientation_constraints = [self._orientation_constraint(pose)]
        request.goal_constraints = [constraints]
        return request

    def _workspace_parameters(self) -> WorkspaceParameters:
        workspace = WorkspaceParameters()
        workspace.header.frame_id = self._string_parameter("target_frame")
        half_size = self._double_parameter("workspace_size") / 2.0
        workspace.min_corner.x = -half_size
        workspace.min_corner.y = -half_size
        workspace.min_corner.z = -0.05
        workspace.max_corner.x = half_size
        workspace.max_corner.y = half_size
        workspace.max_corner.z = self._double_parameter("workspace_size")
        return workspace

    def _position_constraint(self, pose: Pose) -> PositionConstraint:
        constraint = PositionConstraint()
        constraint.header.frame_id = self._string_parameter("target_frame")
        constraint.link_name = self._string_parameter("target_link")
        constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self._double_parameter("position_tolerance_m")]
        constraint.constraint_region.primitives = [sphere]
        constraint.constraint_region.primitive_poses = [pose]
        return constraint

    def _orientation_constraint(self, pose: Pose) -> OrientationConstraint:
        constraint = OrientationConstraint()
        constraint.header.frame_id = self._string_parameter("target_frame")
        constraint.link_name = self._string_parameter("target_link")
        constraint.orientation = pose.orientation
        tolerance = self._double_parameter("orientation_tolerance_rad")
        constraint.absolute_x_axis_tolerance = tolerance
        constraint.absolute_y_axis_tolerance = tolerance
        constraint.absolute_z_axis_tolerance = tolerance
        constraint.weight = 1.0
        return constraint

    def _pose_from_parameters(self, name: str) -> Pose:
        pose = Pose()
        pose.position.x = self._double_parameter(f"{name}_x")
        pose.position.y = self._double_parameter(f"{name}_y")
        pose.position.z = self._double_parameter(f"{name}_z")
        qx = self._double_parameter("horizontal_qx")
        qy = self._double_parameter("horizontal_qy")
        qz = self._double_parameter("horizontal_qz")
        qw = self._double_parameter("horizontal_qw")
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            norm = 1.0
        pose.orientation.x = qx / norm
        pose.orientation.y = qy / norm
        pose.orientation.z = qz / norm
        pose.orientation.w = qw / norm
        return pose

    def _send_action_goal(self, client: ActionClient, goal):
        event = threading.Event()
        state = {"goal_handle": None, "error": None}

        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._store_goal_response(done, event, state)
        )
        if not event.wait(self._double_parameter("server_timeout_sec")):
            return None, "timed out sending action goal"
        return state["goal_handle"], state["error"]

    def _wait_for_action_result(self, goal_handle, timeout_sec: float):
        event = threading.Event()
        state = {"result": None, "error": None}

        future = goal_handle.get_result_async()
        future.add_done_callback(lambda done: self._store_result(done, event, state))
        if not event.wait(timeout_sec):
            return None, "timed out waiting for action result"
        return state["result"], state["error"]

    @staticmethod
    def _store_goal_response(future, event: threading.Event, state: dict) -> None:
        try:
            state["goal_handle"] = future.result()
        except Exception as exc:
            state["error"] = str(exc)
        finally:
            event.set()

    @staticmethod
    def _store_result(future, event: threading.Event, state: dict) -> None:
        try:
            state["result"] = future.result()
        except Exception as exc:
            state["error"] = str(exc)
        finally:
            event.set()

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlowerPickNode()
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

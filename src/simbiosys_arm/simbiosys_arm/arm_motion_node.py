import math
import random
import threading

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from simbiosys_interfaces.action import ExecuteArmMotion


SUCCESS_CODE = MoveItErrorCodes.SUCCESS

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_joint",
]

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


MOVEIT_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: (
        "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"
    ),
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: (
        "UNABLE_TO_AQUIRE_SENSOR_DATA"
    ),
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: (
        "START_STATE_VIOLATES_PATH_CONSTRAINTS"
    ),
    MoveItErrorCodes.START_STATE_INVALID: "START_STATE_INVALID",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: (
        "GOAL_VIOLATES_PATH_CONSTRAINTS"
    ),
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.GOAL_STATE_INVALID: "GOAL_STATE_INVALID",
    MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE: "UNRECOGNIZED_GOAL_TYPE",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: (
        "COLLISION_CHECKING_UNAVAILABLE"
    ),
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
    MoveItErrorCodes.COMMUNICATION_FAILURE: "COMMUNICATION_FAILURE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


class ArmMotionNode(Node):
    """Execute end-effector pose requests through MoveIt."""

    def __init__(self) -> None:
        super().__init__("arm_motion_node")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("planning_group", "mirte_arm")
        self.declare_parameter("target_link", "wrist")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("default_motion_type", "pose")
        self.declare_parameter("position_tolerance", 0.015)
        self.declare_parameter("orientation_tolerance", 0.25)
        self.declare_parameter("orientation_weight", 1.0)
        self.declare_parameter("planning_time_sec", 5.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("pipeline_id", "")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("execute", True)
        self.declare_parameter("replan", True)
        self.declare_parameter("server_timeout_sec", 2.0)
        self.declare_parameter("result_timeout_sec", 30.0)
        self.declare_parameter("workspace_size", 1.0)

        self._move_group_action = self._string_param("move_group_action")
        self._planning_group = self._string_param("planning_group")
        self._target_link = self._string_param("target_link")
        self._target_frame = self._string_param("target_frame")
        self._default_motion_type = self._string_param("default_motion_type")
        self._position_tolerance = self._double_param("position_tolerance")
        self._orientation_tolerance = self._double_param(
            "orientation_tolerance"
        )
        self._orientation_weight = self._double_param("orientation_weight")
        self._planning_time_sec = self._double_param("planning_time_sec")
        self._planning_attempts = self._int_param("planning_attempts")
        self._pipeline_id = self._string_param("pipeline_id")
        self._planner_id = self._string_param("planner_id")
        self._velocity_scaling = self._double_param("velocity_scaling")
        self._acceleration_scaling = self._double_param("acceleration_scaling")
        self._execute = self._bool_param("execute")
        self._replan = self._bool_param("replan")
        self._server_timeout_sec = self._double_param("server_timeout_sec")
        self._result_timeout_sec = self._double_param("result_timeout_sec")
        self._workspace_size = self._double_param("workspace_size")
        self._joint_positions: dict[str, float] = {}
        self._joint_state_lock = threading.Lock()

        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            self._move_group_action,
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )
        self._action_server = ActionServer(
            self,
            ExecuteArmMotion,
            "simbiosys/execute_arm_motion",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            "Arm motion action ready on simbiosys/execute_arm_motion. "
            f"Planning group={self._planning_group}, "
            f"target_link={self._target_link}, "
            f"MoveIt action={self._move_group_action}."
        )

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_param(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool_param(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _goal_callback(
        self,
        goal_request: ExecuteArmMotion.Goal,
    ) -> GoalResponse:
        motion_type = self._motion_type(goal_request.motion_type)
        if motion_type not in ("pose", "position"):
            self.get_logger().warn(
                "Rejecting arm motion goal with unsupported motion_type "
                f"'{goal_request.motion_type}'. Use 'pose' or 'position'."
            )
            return GoalResponse.REJECT

        if not self._pose_is_finite(goal_request.target_pose):
            self.get_logger().warn(
                "Rejecting arm motion goal with non-finite pose"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for arm motion goal")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> ExecuteArmMotion.Result:
        result = ExecuteArmMotion.Result()
        motion_type = self._motion_type(goal_handle.request.motion_type)

        self._publish_feedback(goal_handle, "waiting_for_moveit", 0.05)
        if not self._move_group_client.wait_for_server(
            timeout_sec=self._server_timeout_sec
        ):
            message = (
                f"MoveIt action {self._move_group_action} is not available. "
                "Start it with: "
                "ros2 launch mirte_moveit_config move_group.launch.py"
            )
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        pose = self._normalized_pose(
            goal_handle.request.target_pose,
            motion_type,
        )
        if pose is None:
            message = "Target orientation quaternion is invalid"
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        self._publish_feedback(goal_handle, "sending_moveit_goal", 0.20)
        self.get_logger().info(
            f"Sending MoveIt {motion_type} request for "
            f"{self._target_link} at {self._target_description(pose)}"
        )
        move_group_goal = self._build_move_group_goal(pose, motion_type)
        if move_group_goal is None:
            message = (
                f"No numerical IK solution for {self._target_description(pose)}"
            )
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result
        send_event = threading.Event()
        send_state = {"goal_handle": None, "exception": None}

        send_future = self._move_group_client.send_goal_async(move_group_goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(
                future,
                send_event,
                send_state,
            )
        )
        if not send_event.wait(self._server_timeout_sec):
            message = "Timed out while sending goal to MoveIt"
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        if send_state["exception"] is not None:
            message = (
                "Failed to send goal to MoveIt: "
                f"{send_state['exception']}"
            )
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        moveit_goal_handle = send_state["goal_handle"]
        if moveit_goal_handle is None or not moveit_goal_handle.accepted:
            message = "MoveIt rejected the arm motion goal"
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        self._publish_feedback(goal_handle, "planning_and_executing", 0.50)
        result_event = threading.Event()
        result_state = {"result": None, "exception": None}
        result_future = moveit_goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._on_result(future, result_event, result_state)
        )

        if not result_event.wait(self._result_timeout_sec):
            message = "Timed out waiting for MoveIt result"
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        if result_state["exception"] is not None:
            message = f"MoveIt result failed: {result_state['exception']}"
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        moveit_result = result_state["result"].result
        code = moveit_result.error_code.val
        code_name = MOVEIT_ERROR_NAMES.get(code, f"UNKNOWN_ERROR_{code}")
        if code != SUCCESS_CODE:
            hint = self._moveit_failure_hint(code)
            message = (
                f"MoveIt could not execute {motion_type} target for "
                f"{self._target_link}: {code_name}. "
                f"Target was {self._target_description(pose)}. {hint}"
            )
            self.get_logger().warn(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        self._publish_feedback(goal_handle, "done", 1.0)
        goal_handle.succeed()
        result.success = True
        verb = "Executed" if self._execute else "Planned"
        result.message = (
            f"{verb} {motion_type} target for {self._target_link} in "
            f"{moveit_result.planning_time:.3f}s"
        )
        self.get_logger().info(result.message)
        return result

    def _on_goal_response(
        self,
        future,
        event: threading.Event,
        state: dict,
    ) -> None:
        try:
            state["goal_handle"] = future.result()
        except Exception as exc:
            state["exception"] = exc
        finally:
            event.set()

    def _on_result(self, future, event: threading.Event, state: dict) -> None:
        try:
            state["result"] = future.result()
        except Exception as exc:
            state["exception"] = exc
        finally:
            event.set()

    def _build_move_group_goal(
        self,
        pose: Pose,
        motion_type: str,
    ) -> MoveGroup.Goal | None:
        goal = MoveGroup.Goal()
        goal.request = self._build_motion_plan_request(pose, motion_type)
        if goal.request is None:
            return None

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = not self._execute
        goal.planning_options.replan = self._replan
        goal.planning_options.replan_attempts = 2 if self._replan else 0
        goal.planning_options.replan_delay = 0.25
        return goal

    def _build_motion_plan_request(
        self,
        pose: Pose,
        motion_type: str,
    ) -> MotionPlanRequest | None:
        request = MotionPlanRequest()
        request.group_name = self._planning_group
        request.pipeline_id = self._pipeline_id
        request.planner_id = self._planner_id
        request.num_planning_attempts = self._planning_attempts
        request.allowed_planning_time = self._planning_time_sec
        request.max_velocity_scaling_factor = self._velocity_scaling
        request.max_acceleration_scaling_factor = self._acceleration_scaling
        request.workspace_parameters = self._workspace_parameters()
        request.start_state.is_diff = True
        constraints = self._goal_constraints(pose, motion_type)
        if constraints is None:
            return None
        request.goal_constraints = [constraints]
        return request

    def _workspace_parameters(self) -> WorkspaceParameters:
        workspace = WorkspaceParameters()
        workspace.header.frame_id = self._target_frame
        half_size = self._workspace_size / 2.0
        workspace.min_corner.x = -half_size
        workspace.min_corner.y = -half_size
        workspace.min_corner.z = -0.05
        workspace.max_corner.x = half_size
        workspace.max_corner.y = half_size
        workspace.max_corner.z = self._workspace_size
        return workspace

    def _goal_constraints(
        self,
        pose: Pose,
        motion_type: str,
    ) -> Constraints | None:
        constraints = Constraints()
        constraints.name = f"{self._target_link}_{motion_type}_target"
        if motion_type == "position":
            joint_positions = self._solve_position_ik(pose)
            if joint_positions is None:
                return None
            constraints.joint_constraints = self._joint_constraints(
                joint_positions
            )
            return constraints

        constraints.position_constraints = [self._position_constraint(pose)]
        constraints.orientation_constraints = [
            self._orientation_constraint(pose)
        ]
        return constraints

    def _position_constraint(self, pose: Pose) -> PositionConstraint:
        constraint = PositionConstraint()
        constraint.header.frame_id = self._target_frame
        constraint.link_name = self._target_link
        constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self._position_tolerance]
        constraint.constraint_region.primitives = [sphere]
        constraint.constraint_region.primitive_poses = [pose]
        return constraint

    def _orientation_constraint(self, pose: Pose) -> OrientationConstraint:
        constraint = OrientationConstraint()
        constraint.header.frame_id = self._target_frame
        constraint.link_name = self._target_link
        constraint.orientation = pose.orientation
        constraint.absolute_x_axis_tolerance = self._orientation_tolerance
        constraint.absolute_y_axis_tolerance = self._orientation_tolerance
        constraint.absolute_z_axis_tolerance = self._orientation_tolerance
        constraint.weight = self._orientation_weight
        return constraint

    def _joint_constraints(
        self,
        joint_positions: dict[str, float],
    ) -> list[JointConstraint]:
        constraints = []
        for joint_name in ARM_JOINT_NAMES:
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = joint_positions[joint_name]
            constraint.tolerance_above = 0.03
            constraint.tolerance_below = 0.03
            constraint.weight = 1.0
            constraints.append(constraint)
        return constraints

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_state_lock:
            for name, position in zip(msg.name, msg.position):
                if name in ARM_JOINT_NAMES and math.isfinite(position):
                    self._joint_positions[name] = position

    def _current_arm_positions(self) -> dict[str, float] | None:
        with self._joint_state_lock:
            if not all(name in self._joint_positions for name in ARM_JOINT_NAMES):
                return None
            return {
                name: self._joint_positions[name]
                for name in ARM_JOINT_NAMES
            }

    def _solve_position_ik(self, pose: Pose) -> dict[str, float] | None:
        current = self._current_arm_positions()
        if current is None:
            self.get_logger().warn("No /joint_states arm position received yet")
            return None

        target = (pose.position.x, pose.position.y, pose.position.z)
        if self._position_error(current, target) <= self._position_tolerance:
            return current

        best = current.copy()
        best_error = self._position_error(best, target)
        seeds = [current]
        for _ in range(10):
            seeds.append(
                {
                    name: self._clamp_joint(
                        current[name] + random.uniform(-0.8, 0.8)
                    )
                    for name in ARM_JOINT_NAMES
                }
            )

        for seed in seeds:
            candidate, error = self._coordinate_descent_ik(seed, target)
            if error < best_error:
                best = candidate
                best_error = error

        if best_error > self._position_tolerance:
            self.get_logger().warn(
                "Closest numerical IK solution is %.3fm from target",
                best_error,
            )
            return None
        return best

    def _coordinate_descent_ik(
        self,
        seed: dict[str, float],
        target: tuple[float, float, float],
    ) -> tuple[dict[str, float], float]:
        joint_positions = seed.copy()
        best_error = self._position_error(joint_positions, target)
        step = 0.35
        while step > 0.002:
            improved = False
            for joint_name in ARM_JOINT_NAMES:
                original = joint_positions[joint_name]
                for direction in (1.0, -1.0):
                    joint_positions[joint_name] = self._clamp_joint(
                        original + direction * step
                    )
                    error = self._position_error(joint_positions, target)
                    if error < best_error:
                        best_error = error
                        original = joint_positions[joint_name]
                        improved = True
                joint_positions[joint_name] = original
            if best_error <= self._position_tolerance:
                break
            if not improved:
                step *= 0.5
        return joint_positions, best_error

    def _position_error(
        self,
        joint_positions: dict[str, float],
        target: tuple[float, float, float],
    ) -> float:
        position = self._wrist_position(joint_positions)
        dx = position[0] - target[0]
        dy = position[1] - target[1]
        dz = position[2] - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _wrist_position(
        self,
        joint_positions: dict[str, float],
    ) -> tuple[float, float, float]:
        transform = self._identity_transform()
        for joint in MIRTE_CHAIN:
            transform = self._matmul(
                transform,
                self._origin_transform(
                    joint["origin_xyz"],
                    joint["origin_rpy"],
                ),
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

    def _identity_transform(self) -> list[list[float]]:
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

    def _axis_angle_transform(
        self,
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

    def _rpy_matrix(
        self,
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

    def _matmul(
        self,
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

    def _clamp_joint(self, value: float) -> float:
        return min(JOINT_LIMIT, max(-JOINT_LIMIT, value))

    def _publish_feedback(
        self,
        goal_handle,
        step: str,
        progress: float,
    ) -> None:
        feedback = ExecuteArmMotion.Feedback()
        feedback.current_step = step
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    def _motion_type(self, requested_type: str) -> str:
        motion_type = requested_type.strip().lower()
        if not motion_type:
            return self._default_motion_type.strip().lower()
        return motion_type

    def _target_description(self, pose: Pose) -> str:
        return (
            f"frame={self._target_frame}, "
            f"position=({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, {pose.position.z:.3f}), "
            f"tolerance={self._position_tolerance:.3f}m"
        )

    def _moveit_failure_hint(self, code: int) -> str:
        if code == MoveItErrorCodes.CONTROL_FAILED:
            return (
                "Planning likely succeeded but execution failed; check that "
                "the FollowJointTrajectory controller action is running."
            )
        if code in (
            MoveItErrorCodes.NO_IK_SOLUTION,
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
            MoveItErrorCodes.PLANNING_FAILED,
        ):
            return (
                "The target may be outside the reachable or collision-free "
                "workspace; try execute:=false first to separate planning "
                "from controller execution."
            )
        if code in (
            MoveItErrorCodes.GOAL_IN_COLLISION,
            MoveItErrorCodes.START_STATE_IN_COLLISION,
        ):
            return "MoveIt reported a collision state; check RViz planning scene."
        if code == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return (
                "MoveIt could not transform the target frame; check TF for "
                f"{self._target_frame}."
            )
        return "Check the move_group terminal for the detailed planner reason."

    def _pose_is_finite(self, pose: Pose) -> bool:
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        return all(math.isfinite(value) for value in values)

    def _normalized_pose(self, pose: Pose, motion_type: str) -> Pose | None:
        norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        if norm < 1e-9:
            if motion_type == "pose":
                return None
            norm = 1.0

        normalized = Pose()
        normalized.position = pose.position
        if norm == 1.0 and motion_type == "position":
            normalized.orientation.w = 1.0
        else:
            normalized.orientation.x = pose.orientation.x / norm
            normalized.orientation.y = pose.orientation.y / norm
            normalized.orientation.z = pose.orientation.z / norm
            normalized.orientation.w = pose.orientation.w / norm
        return normalized


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmMotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

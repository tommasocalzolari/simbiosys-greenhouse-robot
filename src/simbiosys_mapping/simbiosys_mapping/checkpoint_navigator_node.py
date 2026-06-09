import copy
import json
import math
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class CheckpointNavigatorNode(Node):
    """Send annotation checkpoints to Nav2 one command at a time."""

    def __init__(self) -> None:
        super().__init__("checkpoint_navigator_node")

        self.declare_parameter("annotations_file", "maps/mirte_map_annotations.json")
        self.declare_parameter("command_topic", "/checkpoint_commands")
        self.declare_parameter("status_topic", "/checkpoint_status")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("nav2_server_timeout_sec", 5.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/map_annotations")
        self.declare_parameter("marker_publish_period", 2.0)
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("initial_pose_publish_period", 1.0)
        self.declare_parameter("initial_pose_publish_count", 10)
        self.declare_parameter("initial_pose_covariance_xy", 0.25)
        self.declare_parameter("initial_pose_covariance_yaw", 0.0685)

        self._route: list[dict[str, Any]] = []
        self._home_pose: PoseStamped | None = None
        self._next_index = 0
        self._active_goal_handle = None
        self._active_target: dict[str, Any] | None = None
        self._state = "starting"
        self._initial_pose_remaining = 0

        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            self._string_parameter("nav2_action_name"),
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String,
            self._string_parameter("status_topic"),
            status_qos,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            self._string_parameter("marker_topic"),
            marker_qos,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self._string_parameter("initial_pose_topic"),
            10,
        )
        self.create_subscription(
            String,
            self._string_parameter("command_topic"),
            self._on_command,
            10,
        )

        self._load_route()
        self._reset_initial_pose_publishing()
        self._state = "waiting"
        self._publish_markers()
        self._publish_initial_pose()
        self._marker_timer = self.create_timer(
            self._double_parameter("marker_publish_period"),
            self._publish_markers,
        )
        self._initial_pose_timer = self.create_timer(
            self._double_parameter("initial_pose_publish_period"),
            self._publish_initial_pose,
        )
        self._publish_status(
            "ready",
            "Checkpoint navigator ready. Send 'next' on /checkpoint_commands.",
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("true", "1", "yes")
        return bool(value)

    def _int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ("next", "go", "advance"):
            self._send_next_checkpoint()
        elif command == "status":
            self._publish_status("status", "Status requested.")
        elif command == "reload":
            if self._active_goal_handle is not None:
                self._publish_status(
                    "busy",
                    "Cancel the active goal before reloading annotations.",
                    error=True,
                )
                return
            self._load_route()
            self._next_index = 0
            self._state = "waiting"
            self._reset_initial_pose_publishing()
            self._publish_markers()
            self._publish_initial_pose()
            self._publish_status("reloaded", "Reloaded annotations and reset route.")
        elif command == "reset":
            if self._active_goal_handle is not None:
                self._publish_status(
                    "busy",
                    "Cancel the active goal before resetting checkpoints.",
                    error=True,
                )
                return
            self._next_index = 0
            self._state = "waiting"
            self._publish_status("reset", "Reset route to home -> first checkpoint.")
        elif command == "skip":
            self._skip_current_checkpoint()
        elif command == "cancel":
            self._cancel_active_goal()
        else:
            self._publish_status(
                "unknown_command",
                "Use one of: next, status, reload, reset, skip, cancel.",
                error=True,
            )

    def _send_next_checkpoint(self) -> None:
        if self._active_goal_handle is not None:
            self._publish_status(
                "busy",
                "A checkpoint goal is already active.",
                error=True,
            )
            return

        if self._next_index >= len(self._route):
            self._state = "complete"
            self._publish_status("complete", "All checkpoints are complete.")
            return

        if not self._nav2_client.wait_for_server(
            timeout_sec=self._double_parameter("nav2_server_timeout_sec")
        ):
            self._state = "waiting"
            self._publish_status(
                "nav2_unavailable",
                "Nav2 NavigateToPose action server is not available.",
                error=True,
            )
            return

        target = self._route[self._next_index]
        goal = NavigateToPose.Goal()
        goal.pose = target["pose"]

        self._active_target = target
        self._state = "sending_goal"
        self._publish_status("sending_goal", f"Sending goal to {target['label']}.")

        send_future = self._nav2_client.send_goal_async(
            goal,
            feedback_callback=self._on_nav2_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            label = self._active_target["label"] if self._active_target else "checkpoint"
            self._active_target = None
            self._active_goal_handle = None
            self._state = "waiting"
            self._publish_status(
                "rejected",
                f"Nav2 rejected goal for {label}.",
                error=True,
            )
            return

        self._active_goal_handle = goal_handle
        self._state = "navigating"
        label = self._active_target["label"] if self._active_target else "checkpoint"
        self._publish_status("navigating", f"Navigating to {label}.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav2_result)

    def _on_nav2_feedback(self, _feedback_msg) -> None:
        pass

    def _on_nav2_result(self, future) -> None:
        result = future.result()
        target = self._active_target
        self._active_goal_handle = None
        self._active_target = None

        label = target["label"] if target is not None else "checkpoint"
        if result is None:
            self._state = "waiting"
            self._publish_status(
                "failed",
                f"Nav2 returned no result for {label}.",
                error=True,
            )
            return

        if int(result.status) == GoalStatus.STATUS_SUCCEEDED:
            self._next_index += 1
            self._state = "waiting"
            self._publish_status(
                "arrived",
                f"Arrived at {label}. Waiting for next command.",
                arrived_target=target,
            )
            return

        self._state = "waiting"
        self._publish_status(
            "failed",
            f"Nav2 failed to reach {label}; send 'next' to retry or 'skip'.",
            error=True,
        )

    def _cancel_active_goal(self) -> None:
        if self._active_goal_handle is None:
            self._publish_status("idle", "No active checkpoint goal to cancel.")
            return

        label = self._active_target["label"] if self._active_target else "checkpoint"
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _future: self._publish_status("cancelled", f"Cancelled {label}.")
        )
        self._active_goal_handle = None
        self._active_target = None
        self._state = "waiting"

    def _skip_current_checkpoint(self) -> None:
        if self._active_goal_handle is not None:
            self._publish_status(
                "busy",
                "Cancel the active goal before skipping.",
                error=True,
            )
            return

        if self._next_index >= len(self._route):
            self._publish_status("complete", "No checkpoint left to skip.")
            return

        skipped = self._route[self._next_index]["label"]
        self._next_index += 1
        self._state = "waiting"
        self._publish_status("skipped", f"Skipped {skipped}.")

    def _load_route(self) -> None:
        annotations_path = Path(self._string_parameter("annotations_file")).expanduser()
        with annotations_path.open("r", encoding="utf-8") as annotations_file:
            annotations = json.load(annotations_file)

        self._home_pose = self._pose_from_dict(annotations["home_pose"])
        route = []
        checkpoints = sorted(
            annotations.get("checkpoints", []),
            key=lambda checkpoint: int(checkpoint.get("checkpoint_id", 0)),
        )
        if checkpoints:
            for checkpoint in checkpoints:
                label = checkpoint.get("label", f"checkpoint_{len(route) + 1}")
                route.append(
                    {
                        "label": label,
                        "pose": self._pose_from_dict(checkpoint["pose"]),
                        "metadata": self._metadata_from_checkpoint(
                            checkpoint,
                            label,
                            len(route) + 1,
                        ),
                    }
                )
        else:
            beds = sorted(
                annotations.get("flower_beds", []),
                key=lambda bed: int(bed.get("bed_id", 0)),
            )
            for bed in beds:
                pose_data = bed.get("start_pose") or self._legacy_bed_pose(bed)
                label = f"flower_bed_{bed.get('bed_id', len(route) + 1)}"
                route.append(
                    {
                        "label": label,
                        "pose": self._pose_from_dict(pose_data),
                        "metadata": self._metadata_from_checkpoint(
                            bed,
                            label,
                            len(route) + 1,
                        ),
                    }
                )

        if not route:
            raise RuntimeError(f"No checkpoints found in {annotations_path}")

        self._route = route
        self.get_logger().info(
            f"Loaded {len(self._route)} checkpoint targets from {annotations_path}"
        )

    def _reset_initial_pose_publishing(self) -> None:
        self._initial_pose_remaining = self._int_parameter(
            "initial_pose_publish_count"
        )

    def _publish_initial_pose(self) -> None:
        if not self._bool_parameter("publish_initial_pose"):
            return
        if self._home_pose is None or self._initial_pose_remaining <= 0:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._home_pose.header.frame_id or "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = copy.deepcopy(self._home_pose.pose)

        covariance_xy = self._double_parameter("initial_pose_covariance_xy")
        covariance_yaw = self._double_parameter("initial_pose_covariance_yaw")
        msg.pose.covariance[0] = covariance_xy
        msg.pose.covariance[7] = covariance_xy
        msg.pose.covariance[35] = covariance_yaw

        self._initial_pose_pub.publish(msg)
        self._initial_pose_remaining -= 1
        self.get_logger().info(
            "Published AMCL initial pose from home_pose "
            f"x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}"
        )

    def _publish_markers(self) -> None:
        if not self._bool_parameter("publish_markers"):
            return

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        marker_id = 0
        if self._home_pose is not None:
            home_markers = self._pose_markers(
                marker_id,
                self._home_pose,
                "home",
                0.0,
                0.75,
                0.15,
            )
            markers.markers.extend(home_markers)
            marker_id += len(home_markers)

        for index, target in enumerate(self._route, start=1):
            checkpoint_markers = self._pose_markers(
                marker_id,
                target["pose"],
                str(index),
                0.1,
                0.25,
                1.0,
            )
            markers.markers.extend(checkpoint_markers)
            marker_id += len(checkpoint_markers)

        self._marker_pub.publish(markers)

    def _pose_markers(
        self,
        marker_id: int,
        pose_msg: PoseStamped,
        label: str,
        red: float,
        green: float,
        blue: float,
    ) -> list[Marker]:
        frame_id = pose_msg.header.frame_id or "map"
        stamp = self.get_clock().now().to_msg()

        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = stamp
        arrow.ns = "checkpoint_pose_arrows"
        arrow.id = marker_id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = copy.deepcopy(pose_msg.pose)
        arrow.scale.x = 0.35
        arrow.scale.y = 0.06
        arrow.scale.z = 0.06
        arrow.color.r = red
        arrow.color.g = green
        arrow.color.b = blue
        arrow.color.a = 1.0

        pin = Marker()
        pin.header.frame_id = frame_id
        pin.header.stamp = stamp
        pin.ns = "checkpoint_pose_pins"
        pin.id = marker_id + 1
        pin.type = Marker.SPHERE
        pin.action = Marker.ADD
        pin.pose.position = copy.deepcopy(pose_msg.pose.position)
        pin.pose.position.z += 0.03
        pin.pose.orientation.w = 1.0
        pin.scale.x = 0.14
        pin.scale.y = 0.14
        pin.scale.z = 0.14
        pin.color.r = red
        pin.color.g = green
        pin.color.b = blue
        pin.color.a = 1.0

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = "checkpoint_pose_labels"
        text.id = marker_id + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = copy.deepcopy(pose_msg.pose.position)
        text.pose.position.z += 0.3
        text.pose.orientation.w = 1.0
        text.scale.z = 0.28
        text.color.r = red
        text.color.g = green
        text.color.b = blue
        text.color.a = 1.0
        text.text = label

        return [arrow, pin, text]

    def _legacy_bed_pose(self, bed: dict[str, Any]) -> dict[str, Any]:
        position = bed.get("start_position", {})
        orientation = bed.get("orientation") or self._quaternion_from_yaw(
            float(bed.get("yaw", 0.0))
        )
        return {
            "frame_id": bed.get("frame_id", "map"),
            "position": position,
            "orientation": orientation,
        }

    def _pose_from_dict(self, pose_data: dict[str, Any]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = pose_data.get("frame_id", "map")
        pose.header.stamp = self.get_clock().now().to_msg()

        position = pose_data["position"]
        orientation = pose_data.get("orientation") or self._quaternion_from_yaw(
            float(pose_data.get("yaw", 0.0))
        )

        pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return pose

    def _metadata_from_checkpoint(
        self,
        checkpoint: dict[str, Any],
        label: str,
        order: int,
    ) -> dict[str, Any]:
        metadata = checkpoint.get("metadata", {})
        if not isinstance(metadata, dict):
            metadata = {}

        bed_id = checkpoint.get("bed_id", metadata.get("bed_id", ""))
        side = checkpoint.get("side", metadata.get("side", ""))
        lane = checkpoint.get("lane", metadata.get("lane", ""))
        scan_position_id = checkpoint.get(
            "scan_position_id",
            metadata.get("scan_position_id", label),
        )
        target_distance_m = checkpoint.get(
            "target_distance_m",
            metadata.get("target_distance_m", None),
        )
        terminal = checkpoint.get("terminal", metadata.get("terminal", False))
        run_perception = checkpoint.get(
            "run_perception",
            metadata.get("run_perception", True),
        )

        return {
            "bed_id": str(bed_id),
            "side": str(side).lower(),
            "lane": str(lane).lower(),
            "scan_position_id": str(scan_position_id or label),
            "order": int(checkpoint.get("order", metadata.get("order", order))),
            "target_distance_m": target_distance_m,
            "terminal": bool(terminal),
            "run_perception": bool(run_perception),
        }

    def _target_payload(self, target: dict[str, Any] | None) -> dict[str, Any] | None:
        if target is None:
            return None
        return {
            "label": target["label"],
            "metadata": copy.deepcopy(target.get("metadata", {})),
            "pose": self._pose_to_dict(target["pose"]),
        }

    def _pose_to_dict(self, pose_msg: PoseStamped) -> dict[str, Any]:
        pose = pose_msg.pose
        return {
            "frame_id": pose_msg.header.frame_id or "map",
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

    def _quaternion_from_yaw(self, yaw: float) -> dict[str, float]:
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(yaw / 2.0),
            "w": math.cos(yaw / 2.0),
        }

    def _publish_status(
        self,
        event: str,
        message: str,
        error: bool = False,
        arrived_target: dict[str, Any] | None = None,
    ) -> None:
        status = {
            "event": event,
            "state": self._state,
            "message": message,
            "error": error,
            "current_start": "home_pose",
            "next_index": self._next_index,
            "route_length": len(self._route),
            "next_target": self._target_payload(self._next_target()),
            "active_target": self._target_payload(self._active_target),
            "arrived_target": self._target_payload(arrived_target),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        if error:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _next_target(self) -> dict[str, Any] | None:
        if self._next_index >= len(self._route):
            return None
        return self._route[self._next_index]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CheckpointNavigatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

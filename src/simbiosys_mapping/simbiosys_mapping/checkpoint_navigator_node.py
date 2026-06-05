import json
import math
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class CheckpointNavigatorNode(Node):
    """Send annotation checkpoints to Nav2 one command at a time."""

    def __init__(self) -> None:
        super().__init__("checkpoint_navigator_node")

        self.declare_parameter("annotations_file", "maps/mirte_map_annotations.json")
        self.declare_parameter("command_topic", "/checkpoint_commands")
        self.declare_parameter("status_topic", "/checkpoint_status")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("nav2_server_timeout_sec", 5.0)

        self._route: list[dict[str, Any]] = []
        self._home_pose: PoseStamped | None = None
        self._next_index = 0
        self._active_goal_handle = None
        self._active_target: dict[str, Any] | None = None
        self._state = "starting"

        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            self._string_parameter("nav2_action_name"),
        )
        self._status_pub = self.create_publisher(
            String,
            self._string_parameter("status_topic"),
            10,
        )
        self.create_subscription(
            String,
            self._string_parameter("command_topic"),
            self._on_command,
            10,
        )

        self._load_route()
        self._state = "waiting"
        self._publish_status(
            "ready",
            "Checkpoint navigator ready. Send 'next' on /checkpoint_commands.",
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

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
                route.append(
                    {
                        "label": checkpoint.get(
                            "label",
                            f"checkpoint_{len(route) + 1}",
                        ),
                        "pose": self._pose_from_dict(checkpoint["pose"]),
                    }
                )
        else:
            beds = sorted(
                annotations.get("flower_beds", []),
                key=lambda bed: int(bed.get("bed_id", 0)),
            )
            for bed in beds:
                pose_data = bed.get("start_pose") or self._legacy_bed_pose(bed)
                route.append(
                    {
                        "label": f"flower_bed_{bed.get('bed_id', len(route) + 1)}",
                        "pose": self._pose_from_dict(pose_data),
                    }
                )

            final_pose = annotations.get("final_pose")
            if final_pose is not None:
                route.append(
                    {"label": "final_pose", "pose": self._pose_from_dict(final_pose)}
                )

        if not route:
            raise RuntimeError(f"No checkpoints found in {annotations_path}")

        self._route = route
        self.get_logger().info(
            f"Loaded {len(self._route)} checkpoint targets from {annotations_path}"
        )

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
    ) -> None:
        status = {
            "event": event,
            "state": self._state,
            "message": message,
            "error": error,
            "current_start": "home_pose",
            "next_index": self._next_index,
            "route_length": len(self._route),
            "next_target": self._next_target_label(),
            "active_target": (
                self._active_target["label"] if self._active_target is not None else None
            ),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        if error:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _next_target_label(self) -> str | None:
        if self._next_index >= len(self._route):
            return None
        return self._route[self._next_index]["label"]


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

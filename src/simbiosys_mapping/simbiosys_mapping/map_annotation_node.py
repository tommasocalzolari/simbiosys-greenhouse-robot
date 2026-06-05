import copy
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class MapAnnotationNode(Node):
    """Publish a saved map read-only and collect checkpoint annotations."""

    def __init__(self) -> None:
        super().__init__("map_annotation_node")

        self.declare_parameter("map_yaml", "maps/mirte_map.yaml")
        self.declare_parameter("annotations_file", "maps/mirte_map_annotations.json")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("annotation_markers_topic", "/map_annotations")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("annotation_frame", "map")
        self.declare_parameter("start_annotation_on_startup", True)

        self._map: OccupancyGrid | None = None
        self._annotation_stage = "idle"
        self._home_pose: PoseStamped | None = None
        self._checkpoints: list[dict[str, Any]] = []

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(
            OccupancyGrid,
            self._string_parameter("map_topic"),
            map_qos,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            self._string_parameter("annotation_markers_topic"),
            marker_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self._string_parameter("initial_pose_topic"),
            self._on_initial_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self._string_parameter("goal_pose_topic"),
            self._on_goal_pose,
            10,
        )

        self.create_service(Trigger, "~/reload_map", self._on_reload_map)
        self.create_service(Trigger, "~/start_annotation", self._on_start_annotation)
        self.create_service(Trigger, "~/save_annotations", self._on_save_annotations)
        self.create_service(Trigger, "~/finish_annotation", self._on_finish_annotation)
        self.create_service(Trigger, "~/reset_annotation", self._on_reset_annotation)
        self.create_service(
            Trigger,
            "~/undo_last_checkpoint",
            self._on_undo_last_checkpoint,
        )

        self._load_and_publish_map()
        if self._bool_parameter("start_annotation_on_startup"):
            self._start_annotation_session()

        self.get_logger().info(
            "Map annotation node ready. This node does not modify the map file."
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _on_reload_map(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            self._load_and_publish_map()
            response.success = True
            response.message = "Reloaded and republished the saved map."
        except Exception as exc:
            response.success = False
            response.message = f"Failed to reload map: {exc}"
        return response

    def _on_start_annotation(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._start_annotation_session()
        response.success = True
        response.message = (
            "Annotation started. Use 2D Pose Estimate for home, then 2D Goal "
            "Pose for each checkpoint in order."
        )
        self.get_logger().info(response.message)
        return response

    def _on_finish_annotation(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        return self._save_annotations_response(response)

    def _on_save_annotations(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        return self._save_annotations_response(response)

    def _save_annotations_response(
        self,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._home_pose is None:
            response.success = False
            response.message = "Set the home pose before finishing."
            return response
        if not self._checkpoints:
            response.success = False
            response.message = "Set at least one checkpoint before finishing."
            return response

        annotations_path = self._save_annotations()
        response.success = True
        response.message = (
            f"Saved annotations to {annotations_path}. "
            "Annotation remains active, so you can keep adding checkpoints."
        )
        self.get_logger().info(response.message)
        return response

    def _on_reset_annotation(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._annotation_stage = "idle"
        self._home_pose = None
        self._checkpoints = []
        self._publish_annotation_markers()
        response.success = True
        response.message = "Cleared current annotations."
        return response

    def _on_undo_last_checkpoint(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._checkpoints:
            response.success = False
            response.message = "No checkpoint annotations to undo."
            return response

        removed = self._checkpoints.pop()
        self._publish_annotation_markers()
        self._save_annotations()
        response.success = True
        response.message = f"Removed {removed['label']}."
        return response

    def _start_annotation_session(self) -> None:
        self._home_pose = None
        self._checkpoints = []
        self._annotation_stage = "home"
        self._publish_annotation_markers()

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._annotation_stage != "home":
            return

        home_pose = PoseStamped()
        home_pose.header = copy.deepcopy(msg.header)
        home_pose.pose = copy.deepcopy(msg.pose.pose)
        self._home_pose = home_pose
        self._annotation_stage = "checkpoints"
        self._publish_annotation_markers()
        self._save_annotations()
        self.get_logger().info(
            "Saved home pose. Now add checkpoints with 2D Goal Pose."
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self._annotation_stage != "checkpoints":
            return

        checkpoint_id = len(self._checkpoints) + 1
        pose_dict = self._pose_to_dict(msg)
        checkpoint = {
            "checkpoint_id": checkpoint_id,
            "label": f"checkpoint_{checkpoint_id}",
            "frame_id": pose_dict["frame_id"],
            "pose": pose_dict,
            "position": pose_dict["position"],
            "orientation": pose_dict["orientation"],
            "yaw": pose_dict["yaw"],
        }
        self._checkpoints.append(checkpoint)
        self._publish_annotation_markers()
        self._save_annotations()
        self.get_logger().info(
            f"Saved {checkpoint['label']} with orientation. "
            "Select 2D Goal Pose again if RViz switches tools."
        )

    def _load_and_publish_map(self) -> None:
        map_yaml = Path(self._string_parameter("map_yaml")).expanduser()
        self._map = self._load_map_from_yaml(map_yaml)
        self._map_pub.publish(self._map)
        self.get_logger().info(f"Published map read-only from {map_yaml}")

    def _load_map_from_yaml(self, yaml_path: Path) -> OccupancyGrid:
        if not yaml_path.exists():
            raise FileNotFoundError(f"Map YAML does not exist: {yaml_path}")

        with yaml_path.open("r", encoding="utf-8") as yaml_file:
            metadata = yaml.safe_load(yaml_file)

        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"Map image does not exist: {image_path}")

        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.25))

        pixel = image.astype(np.float32)
        probability = pixel / 255.0 if negate else (255.0 - pixel) / 255.0
        occupancy_image = np.full(image.shape, -1, dtype=np.int16)
        occupancy_image[probability > occupied_thresh] = 100
        occupancy_image[probability < free_thresh] = 0
        occupancy_grid = np.flipud(occupancy_image)

        msg = OccupancyGrid()
        msg.header.frame_id = self._string_parameter("annotation_frame")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self._metadata_from_yaml(metadata, image.shape[1], image.shape[0])
        msg.data = occupancy_grid.astype(np.int8).reshape(-1).tolist()
        return msg

    def _metadata_from_yaml(
        self,
        metadata: dict[str, Any],
        width: int,
        height: int,
    ) -> MapMetaData:
        info = MapMetaData()
        info.width = width
        info.height = height
        info.resolution = float(metadata["resolution"])

        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        info.origin.position.x = float(origin[0])
        info.origin.position.y = float(origin[1])
        info.origin.position.z = 0.0
        info.origin.orientation = self._quaternion_msg_from_yaw(float(origin[2]))
        return info

    def _save_annotations(self) -> Path:
        annotations_path = Path(
            self._string_parameter("annotations_file")
        ).expanduser()
        annotations_path.parent.mkdir(parents=True, exist_ok=True)

        checkpoints = self._checkpoints
        annotations = {
            "frame_id": self._string_parameter("annotation_frame"),
            "home_pose": self._pose_to_dict(self._home_pose),
            "checkpoints": checkpoints,
            # Compatibility for older code that expected flower_beds.
            "flower_beds": [
                {
                    "bed_id": checkpoint["checkpoint_id"],
                    "frame_id": checkpoint["frame_id"],
                    "start_pose": checkpoint["pose"],
                    "start_position": checkpoint["position"],
                    "orientation": checkpoint["orientation"],
                    "yaw": checkpoint["yaw"],
                }
                for checkpoint in checkpoints
            ],
            "final_pose": checkpoints[-1]["pose"] if checkpoints else None,
        }
        annotations_path.write_text(
            json.dumps(annotations, indent=2),
            encoding="utf-8",
        )
        return annotations_path

    def _pose_to_dict(self, pose_msg: PoseStamped | None) -> dict[str, Any] | None:
        if pose_msg is None:
            return None

        pose = pose_msg.pose
        return {
            "frame_id": pose_msg.header.frame_id
            or self._string_parameter("annotation_frame"),
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
            "yaw": self._yaw_from_quaternion(pose.orientation),
        }

    def _publish_annotation_markers(self) -> None:
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        marker_id = 0
        if self._home_pose is not None:
            pose_markers = self._pose_markers(
                marker_id,
                self._home_pose,
                "home",
                0.0,
                0.7,
                0.1,
            )
            markers.markers.extend(pose_markers)
            marker_id += len(pose_markers)

        for checkpoint in self._checkpoints:
            checkpoint_markers = self._checkpoint_markers(marker_id, checkpoint)
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
        frame_id = pose_msg.header.frame_id or self._string_parameter("annotation_frame")
        stamp = self.get_clock().now().to_msg()

        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = stamp
        arrow.ns = "map_pose_annotations"
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
        pin.ns = "map_pin_annotations"
        pin.id = marker_id + 1
        pin.type = Marker.SPHERE
        pin.action = Marker.ADD
        pin.pose.position = copy.deepcopy(pose_msg.pose.position)
        pin.pose.position.z += 0.03
        pin.pose.orientation.w = 1.0
        pin.scale.x = 0.12
        pin.scale.y = 0.12
        pin.scale.z = 0.12
        pin.color.r = red
        pin.color.g = green
        pin.color.b = blue
        pin.color.a = 1.0

        text = self._text_marker(
            marker_id + 2,
            frame_id,
            stamp,
            label,
            pose_msg.pose.position,
            red,
            green,
            blue,
        )
        return [arrow, pin, text]

    def _checkpoint_markers(
        self,
        marker_id: int,
        checkpoint: dict[str, Any],
    ) -> list[Marker]:
        pose = PoseStamped()
        pose.header.frame_id = checkpoint["frame_id"]
        pose.pose.position.x = checkpoint["position"]["x"]
        pose.pose.position.y = checkpoint["position"]["y"]
        pose.pose.position.z = checkpoint["position"]["z"]
        pose.pose.orientation.x = checkpoint["orientation"]["x"]
        pose.pose.orientation.y = checkpoint["orientation"]["y"]
        pose.pose.orientation.z = checkpoint["orientation"]["z"]
        pose.pose.orientation.w = checkpoint["orientation"]["w"]
        return self._pose_markers(
            marker_id,
            pose,
            str(checkpoint["checkpoint_id"]),
            0.1,
            0.25,
            1.0,
        )

    def _text_marker(
        self,
        marker_id: int,
        frame_id: str,
        stamp,
        text: str,
        point: Point,
        red: float,
        green: float,
        blue: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "map_text_annotations"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z + 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.24
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.text = text
        return marker

    def _quaternion_msg_from_yaw(self, yaw: float):
        from geometry_msgs.msg import Quaternion

        orientation = Quaternion()
        orientation.z = math.sin(yaw / 2.0)
        orientation.w = math.cos(yaw / 2.0)
        return orientation

    def _yaw_from_quaternion(self, orientation) -> float:
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapAnnotationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

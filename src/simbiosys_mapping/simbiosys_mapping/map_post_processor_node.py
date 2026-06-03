import copy
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class MapPostProcessorNode(Node):
    """Clean a saved map and collect RViz map annotations."""

    def __init__(self) -> None:
        super().__init__("map_post_processor_node")

        self.declare_parameter("map_yaml", "maps/mirte_map.yaml")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("annotation_name", "mirte_map_annotations")
        self.declare_parameter("occupied_thresh", 0.65)
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("min_occupied_cluster_size", 2)
        self.declare_parameter("straighten_kernel_size", 5)
        self.declare_parameter("closed_obstacle_min_area", 8)
        self.declare_parameter("closed_obstacle_max_area_ratio", 0.15)
        self.declare_parameter("rectangularize_closed_obstacles", True)
        self.declare_parameter("process_on_startup", True)
        self.declare_parameter("start_annotation_after_processing", True)
        self.declare_parameter("annotation_frame", "map")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("annotation_markers_topic", "/map_annotations")

        self._processed_map: OccupancyGrid | None = None
        self._annotation_stage = "idle"
        self._home_pose: PoseStamped | None = None
        self._final_pose: PoseStamped | None = None
        self._flower_beds: list[dict[str, Any]] = []
        self._startup_timer = None

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
        self.create_subscription(
            PointStamped,
            self._string_parameter("clicked_point_topic"),
            self._on_clicked_point,
            10,
        )

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

        self.create_service(Trigger, "~/process_map", self._on_process_map)
        self.create_service(Trigger, "~/start_annotation", self._on_start_annotation)
        self.create_service(Trigger, "~/finish_annotation", self._on_finish_annotation)
        self.create_service(Trigger, "~/reset_annotation", self._on_reset_annotation)
        self.create_service(Trigger, "~/undo_last_bed", self._on_undo_last_bed)

        if self._bool_parameter("process_on_startup"):
            self._startup_timer = self.create_timer(1.0, self._process_on_startup)

        self.get_logger().info(
            "Offline map post processor ready. Process and overwrite the saved map with: "
            "ros2 service call /map_post_processor_node/process_map "
            'std_srvs/srv/Trigger "{}"'
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _integer_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _on_process_map(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            map_yaml = Path(self._string_parameter("map_yaml")).expanduser()
            source_map = self._load_map_from_yaml(map_yaml)

            cleaned_map = self._clean_map(source_map)
            yaml_path = self._save_map(cleaned_map, map_yaml)
            self._processed_map = cleaned_map
            self._map_pub.publish(cleaned_map)

            if self._bool_parameter("start_annotation_after_processing"):
                self._start_annotation_session()

            response.success = True
            response.message = (
                f"Processed and overwrote {yaml_path}. "
                f"Published cleaned map on {self._string_parameter('map_topic')}."
            )
        except Exception as exc:
            response.success = False
            response.message = f"Failed to process map: {exc}"
        return response

    def _process_on_startup(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
        request = Trigger.Request()
        response = Trigger.Response()
        response = self._on_process_map(request, response)
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)

    def _clean_map(self, msg: OccupancyGrid) -> OccupancyGrid:
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))

        occupied_limit = int(self._double_parameter("occupied_thresh") * 100)
        occupied = np.where(grid >= occupied_limit, 255, 0).astype(np.uint8)

        occupied = self._remove_small_occupied_clusters(occupied)
        occupied = self._straighten_occupied_edges(occupied)
        occupied = self._remove_small_occupied_clusters(occupied)
        occupied = self._fill_closed_obstacles(occupied)

        cleaned_grid = grid.copy()
        originally_occupied = grid >= occupied_limit
        cleaned_grid[np.logical_and(originally_occupied, occupied == 0)] = 0
        cleaned_grid[occupied > 0] = 100

        cleaned_msg = OccupancyGrid()
        cleaned_msg.header = copy.deepcopy(msg.header)
        cleaned_msg.header.stamp = self.get_clock().now().to_msg()
        cleaned_msg.info = copy.deepcopy(msg.info)
        cleaned_msg.data = cleaned_grid.astype(np.int8).reshape(-1).tolist()
        return cleaned_msg

    def _remove_small_occupied_clusters(self, occupied: np.ndarray) -> np.ndarray:
        min_size = max(1, self._integer_parameter("min_occupied_cluster_size"))
        output = occupied.copy()
        labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            output,
            connectivity=8,
        )
        for label in range(1, labels_count):
            area = stats[label, cv2.CC_STAT_AREA]
            if area < min_size:
                output[labels == label] = 0
        return output

    def _straighten_occupied_edges(self, occupied: np.ndarray) -> np.ndarray:
        kernel_size = max(1, self._integer_parameter("straighten_kernel_size"))
        if kernel_size <= 1:
            return occupied

        horizontal_kernel = np.ones((1, kernel_size), dtype=np.uint8)
        vertical_kernel = np.ones((kernel_size, 1), dtype=np.uint8)
        horizontal = cv2.morphologyEx(
            occupied,
            cv2.MORPH_CLOSE,
            horizontal_kernel,
        )
        vertical = cv2.morphologyEx(
            occupied,
            cv2.MORPH_CLOSE,
            vertical_kernel,
        )
        combined = cv2.bitwise_or(horizontal, vertical)
        return cv2.morphologyEx(
            combined,
            cv2.MORPH_OPEN,
            np.ones((2, 2), np.uint8),
        )

    def _fill_closed_obstacles(self, occupied: np.ndarray) -> np.ndarray:
        output = occupied.copy()
        height, width = output.shape
        map_area = float(width * height)
        min_area = self._integer_parameter("closed_obstacle_min_area")
        max_area_ratio = self._double_parameter("closed_obstacle_max_area_ratio")
        rectangularize = self._bool_parameter("rectangularize_closed_obstacles")

        contours, _ = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            bbox_area = float(w * h)

            if contour_area < min_area:
                continue
            if bbox_area > map_area * max_area_ratio:
                continue
            if x <= 0 or y <= 0 or x + w >= width - 1 or y + h >= height - 1:
                continue
            if w < 3 or h < 3:
                continue

            if rectangularize:
                cv2.rectangle(output, (x, y), (x + w - 1, y + h - 1), 255, -1)
            else:
                cv2.drawContours(output, [contour], -1, 255, thickness=cv2.FILLED)

        return output

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
        info.origin.orientation = self._quaternion_from_yaw(float(origin[2]))
        return info

    def _save_map(self, msg: OccupancyGrid, yaml_path: Path) -> Path:
        occupied_thresh = self._double_parameter("occupied_thresh")
        free_thresh = self._double_parameter("free_thresh")

        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = yaml_path.with_suffix(".pgm")

        self._write_pgm(msg, pgm_path, occupied_thresh, free_thresh)
        self._write_yaml(msg, yaml_path, pgm_path.name, occupied_thresh, free_thresh)
        return yaml_path

    def _write_pgm(
        self,
        msg: OccupancyGrid,
        path: Path,
        occupied_thresh: float,
        free_thresh: float,
    ) -> None:
        width = msg.info.width
        height = msg.info.height
        occupied_limit = int(occupied_thresh * 100)
        free_limit = int(free_thresh * 100)

        with path.open("wb") as pgm_file:
            pgm_file.write(
                f"P5\n# CREATOR: simbiosys_mapping map_post_processor_node\n"
                f"{width} {height}\n255\n".encode("ascii")
            )
            for image_y in range(height):
                map_y = height - image_y - 1
                for x in range(width):
                    value = msg.data[x + (map_y * width)]
                    if value < 0:
                        pixel_value = 205
                    elif value >= occupied_limit:
                        pixel_value = 0
                    elif value <= free_limit:
                        pixel_value = 254
                    else:
                        pixel_value = 205
                    pgm_file.write(bytes([pixel_value]))

    def _write_yaml(
        self,
        msg: OccupancyGrid,
        path: Path,
        image_name: str,
        occupied_thresh: float,
        free_thresh: float,
    ) -> None:
        origin = msg.info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        path.write_text(
            "\n".join(
                [
                    f"image: {image_name}",
                    "mode: trinary",
                    f"resolution: {msg.info.resolution}",
                    (
                        "origin: "
                        f"[{origin.position.x}, {origin.position.y}, {yaw}]"
                    ),
                    "negate: 0",
                    f"occupied_thresh: {occupied_thresh}",
                    f"free_thresh: {free_thresh}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

    def _on_start_annotation(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._start_annotation_session()
        response.success = True
        response.message = (
            "Annotation started. In RViz: set home with 2D Pose Estimate, "
            "set final with 2D Goal Pose, then click bed starts with Publish Point."
        )
        self.get_logger().info(response.message)
        return response

    def _start_annotation_session(self) -> None:
        self._home_pose = None
        self._final_pose = None
        self._flower_beds = []
        self._annotation_stage = "home"
        self._publish_annotation_markers()

    def _on_finish_annotation(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._home_pose is None or self._final_pose is None:
            response.success = False
            response.message = "Set home pose and final pose before finishing."
            return response

        annotation_path = self._save_annotations()
        self._annotation_stage = "idle"
        response.success = True
        response.message = f"Saved annotations to {annotation_path}"
        self.get_logger().info(response.message)
        return response

    def _on_reset_annotation(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._annotation_stage = "idle"
        self._home_pose = None
        self._final_pose = None
        self._flower_beds = []
        self._publish_annotation_markers()

        response.success = True
        response.message = "Cleared current annotations."
        return response

    def _on_undo_last_bed(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._flower_beds:
            response.success = False
            response.message = "No flower bed annotations to undo."
            return response

        removed = self._flower_beds.pop()
        self._publish_annotation_markers()
        self._save_annotations()
        response.success = True
        response.message = f"Removed flower bed {removed['bed_id']}."
        return response

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._annotation_stage != "home":
            return

        home_pose = PoseStamped()
        home_pose.header = copy.deepcopy(msg.header)
        home_pose.pose = copy.deepcopy(msg.pose.pose)
        self._home_pose = home_pose
        self._annotation_stage = "final"
        self._publish_annotation_markers()
        self._save_annotations()
        self.get_logger().info("Saved home pose. Now set final pose with 2D Goal Pose.")

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self._annotation_stage != "final":
            return

        self._final_pose = copy.deepcopy(msg)
        self._annotation_stage = "beds"
        self._publish_annotation_markers()
        self._save_annotations()
        self.get_logger().info(
            "Saved final pose. Now click flower bed start positions with Publish Point."
        )

    def _on_clicked_point(self, msg: PointStamped) -> None:
        if self._annotation_stage != "beds":
            return

        bed_id = len(self._flower_beds) + 1
        self._flower_beds.append(
            {
                "bed_id": bed_id,
                "frame_id": msg.header.frame_id or self._string_parameter("annotation_frame"),
                "start_position": {
                    "x": msg.point.x,
                    "y": msg.point.y,
                    "z": msg.point.z,
                },
            }
        )
        self._publish_annotation_markers()
        self._save_annotations()
        self.get_logger().info(f"Saved flower bed {bed_id} start position.")

    def _save_annotations(self) -> Path:
        map_yaml = Path(self._string_parameter("map_yaml")).expanduser()
        output_dir = map_yaml.parent
        annotation_name = self._string_parameter("annotation_name")
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{annotation_name}.json"

        annotations = {
            "frame_id": self._string_parameter("annotation_frame"),
            "home_pose": self._pose_to_dict(self._home_pose),
            "final_pose": self._pose_to_dict(self._final_pose),
            "flower_beds": self._flower_beds,
        }
        output_path.write_text(
            json.dumps(annotations, indent=2),
            encoding="utf-8",
        )
        return output_path

    def _pose_to_dict(self, pose_msg: PoseStamped | None) -> dict[str, Any] | None:
        if pose_msg is None:
            return None

        pose = pose_msg.pose
        return {
            "frame_id": pose_msg.header.frame_id or self._string_parameter("annotation_frame"),
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
            markers.markers.extend(
                self._pose_markers(marker_id, self._home_pose, "home", 0.0, 0.7, 0.1)
            )
            marker_id += 2

        if self._final_pose is not None:
            markers.markers.extend(
                self._pose_markers(marker_id, self._final_pose, "final", 0.9, 0.1, 0.1)
            )
            marker_id += 2

        for bed in self._flower_beds:
            markers.markers.extend(self._bed_markers(marker_id, bed))
            marker_id += 2

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

        text = self._text_marker(
            marker_id + 1,
            frame_id,
            stamp,
            label,
            pose_msg.pose.position,
            red,
            green,
            blue,
        )
        return [arrow, text]

    def _bed_markers(self, marker_id: int, bed: dict[str, Any]) -> list[Marker]:
        frame_id = bed["frame_id"]
        stamp = self.get_clock().now().to_msg()
        point = Point()
        point.x = bed["start_position"]["x"]
        point.y = bed["start_position"]["y"]
        point.z = bed["start_position"]["z"]

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "flower_bed_start_annotations"
        sphere.id = marker_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.14
        sphere.scale.y = 0.14
        sphere.scale.z = 0.14
        sphere.color.r = 0.1
        sphere.color.g = 0.25
        sphere.color.b = 1.0
        sphere.color.a = 1.0

        text = self._text_marker(
            marker_id + 1,
            frame_id,
            stamp,
            str(bed["bed_id"]),
            point,
            0.1,
            0.25,
            1.0,
        )
        return [sphere, text]

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

    def _quaternion_from_yaw(self, yaw: float):
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
    node = MapPostProcessorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

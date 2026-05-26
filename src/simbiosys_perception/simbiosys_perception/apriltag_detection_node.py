import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, String

from simbiosys_interfaces.msg import BedObservation, DetectedTag


class AprilTagDetectionNode(Node):
    """Detect AprilTag 36h11 markers and publish visible tag IDs."""

    def __init__(self) -> None:
        super().__init__("apriltag_detection_node")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("min_tag_area", 500.0)

        self._camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        self._use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )
        self._subscribed_camera_topic = self._camera_topic
        if self._use_compressed:
            self._subscribed_camera_topic = f"{self._camera_topic}/compressed"
        self._min_tag_area = (
            self.get_parameter("min_tag_area").get_parameter_value().double_value
        )

        self._bridge = CvBridge()
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        parameters = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, parameters)

        self._tags_publisher = self.create_publisher(
            String,
            "/simbiosys/detected_tags",
            10,
        )
        self._current_bed_publisher = self.create_publisher(
            Int32,
            "/simbiosys/current_bed_id",
            10,
        )
        self._bed_observation_publisher = self.create_publisher(
            BedObservation,
            "simbiosys/bed_observation",
            10,
        )
        image_msg_type = CompressedImage if self._use_compressed else Image
        self._image_subscription = self.create_subscription(
            image_msg_type,
            self._subscribed_camera_topic,
            self._on_image,
            10,
        )

        self.get_logger().info(
            f"AprilTag detection listening on {self._subscribed_camera_topic}, "
            "publishing simbiosys/bed_observation with legacy tag topics"
        )

    def _on_image(self, image_msg: Image | CompressedImage) -> None:
        frame = self._image_msg_to_frame(image_msg)
        if frame is None:
            return

        detections = self._detect_tags(frame)
        self._publish_detections(detections)

    def _image_msg_to_frame(
        self,
        image_msg: Image | CompressedImage,
    ) -> np.ndarray | None:
        if self._use_compressed:
            image_buffer = np.frombuffer(image_msg.data, np.uint8)
            frame = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning("Could not decode compressed camera image")
            return frame

        try:
            return self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert camera image: {exc}")
            return None

    def _detect_tags(self, frame: np.ndarray) -> list[dict[str, float | int]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect_markers(gray)
        if ids is None:
            return []

        detections = []
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            points = tag_corners.reshape(4, 2)
            area = self._tag_bounding_box_area(points)
            if area < self._min_tag_area:
                continue

            center = points.mean(axis=0)
            detections.append(
                {
                    "id": int(tag_id),
                    "center_x": int(round(float(center[0]))),
                    "center_y": int(round(float(center[1]))),
                    "area": area,
                }
            )

        return detections

    def _detect_markers(self, gray: np.ndarray):
        corners, ids, _ = self._detector.detectMarkers(gray)
        return corners, ids

    def _tag_bounding_box_area(self, points: np.ndarray) -> float:
        x_min = float(points[:, 0].min())
        x_max = float(points[:, 0].max())
        y_min = float(points[:, 1].min())
        y_max = float(points[:, 1].max())
        return (x_max - x_min) * (y_max - y_min)

    def _publish_detections(self, detections: list[dict[str, float | int]]) -> None:
        payload_tags = [
            {
                "id": detection["id"],
                "center_x": detection["center_x"],
                "center_y": detection["center_y"],
            }
            for detection in detections
        ]

        tags_msg = String()
        tags_msg.data = json.dumps({"tags": payload_tags})
        self._tags_publisher.publish(tags_msg)

        current_bed_msg = Int32()
        if detections:
            closest_detection = max(
                detections,
                key=lambda detection: detection["area"],
            )
            current_bed_msg.data = int(closest_detection["id"])
            detected_ids = [detection["id"] for detection in detections]
            self.get_logger().info(f"Detected AprilTag IDs: {detected_ids}")
        else:
            current_bed_msg.data = -1
            self.get_logger().info("Detected AprilTag IDs: []")

        self._current_bed_publisher.publish(current_bed_msg)
        self._publish_bed_observation(detections, current_bed_msg.data)

    def _publish_bed_observation(
        self,
        detections: list[dict[str, float | int]],
        bed_id: int,
    ) -> None:
        msg = BedObservation()
        msg.bed_id = bed_id
        msg.visible = bed_id >= 0
        msg.message = (
            f"Visible bed tag {bed_id}"
            if msg.visible
            else "No AprilTag bed marker visible"
        )

        msg.tags = []
        for detection in detections:
            tag = DetectedTag()
            tag.id = int(detection["id"])
            tag.center_px = Point(
                x=float(detection["center_x"]),
                y=float(detection["center_y"]),
                z=0.0,
            )
            tag.area = float(detection["area"])
            tag.confidence = 1.0
            msg.tags.append(tag)

        self._bed_observation_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

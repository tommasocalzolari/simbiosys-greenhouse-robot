from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Int32, String


@dataclass(frozen=True)
class BugCandidate:
    bbox: tuple[int, int, int, int]
    contour_area: float
    dark_pixel_ratio: float
    blob_ratio: float


class BugDetectionNode(Node):
    """Detect bug tags: white square cards with a dark spider silhouette."""

    def __init__(self) -> None:
        super().__init__("bug_detection_node")
        self.declare_parameter("camera_topic", "/gripper_camera/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("min_tag_area", 300.0)
        self.declare_parameter("max_tag_area", 50000.0)
        self.declare_parameter("dark_pixel_ratio", 0.15)
        self.declare_parameter("blob_ratio_threshold", 0.4)

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
        self._max_tag_area = (
            self.get_parameter("max_tag_area").get_parameter_value().double_value
        )
        self._dark_pixel_ratio = (
            self.get_parameter("dark_pixel_ratio").get_parameter_value().double_value
        )
        self._blob_ratio_threshold = (
            self.get_parameter("blob_ratio_threshold")
            .get_parameter_value()
            .double_value
        )

        self._bridge = CvBridge()
        self._current_bed_id = -1

        self._bug_detected_publisher = self.create_publisher(
            Bool,
            "/simbiosys/bug_detected",
            10,
        )
        self._bug_count_publisher = self.create_publisher(
            Int32,
            "/simbiosys/bug_count",
            10,
        )
        self._debug_publisher = self.create_publisher(
            String,
            "/simbiosys/bug_debug",
            10,
        )
        image_msg_type = CompressedImage if self._use_compressed else Image
        self._image_subscription = self.create_subscription(
            image_msg_type,
            self._subscribed_camera_topic,
            self._on_image,
            10,
        )
        self._bed_id_subscription = self.create_subscription(
            Int32,
            "/simbiosys/current_bed_id",
            self._on_current_bed_id,
            10,
        )

        self.get_logger().info(
            f"Bug detection listening on {self._subscribed_camera_topic}, "
            "publishing /simbiosys/bug_detected, /simbiosys/bug_count, "
            "and /simbiosys/bug_debug"
        )

    def _on_current_bed_id(self, msg: Int32) -> None:
        self._current_bed_id = msg.data

    def _on_image(self, image_msg: Image | CompressedImage) -> None:
        frame = self._image_msg_to_frame(image_msg)
        if frame is None:
            return

        detections = self._detect_bugs(frame)
        self._publish_results(len(detections))

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

    def _detect_bugs(self, frame: np.ndarray) -> list[BugCandidate]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 140], dtype=np.uint8),
            np.array([179, 50, 255], dtype=np.uint8),
        )
        white_mask = self._clean_mask(white_mask)

        contours, _ = cv2.findContours(
            white_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        detections = []
        for contour in contours:
            if not self._is_square_tag_contour(contour):
                continue

            candidate = self._candidate_from_contour(gray, contour)
            if candidate is not None:
                detections.append(candidate)

        return detections

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _is_square_tag_contour(self, contour: np.ndarray) -> bool:
        area = cv2.contourArea(contour)
        if area < self._min_tag_area or area > self._max_tag_area:
            return False

        x, y, width, height = cv2.boundingRect(contour)
        if height == 0:
            return False

        aspect_ratio = width / float(height)
        return 0.7 <= aspect_ratio <= 1.4

    def _candidate_from_contour(
        self,
        gray: np.ndarray,
        contour: np.ndarray,
    ) -> BugCandidate | None:
        x, y, width, height = cv2.boundingRect(contour)
        crop = gray[y : y + height, x : x + width]
        if crop.size == 0:
            return None

        dark_mask = cv2.adaptiveThreshold(
            crop,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            10,
        )
        total_dark_pixels = int(cv2.countNonZero(dark_mask))
        dark_pixel_ratio = total_dark_pixels / float(crop.size)
        if dark_pixel_ratio < self._dark_pixel_ratio:
            return None

        dark_contours, _ = cv2.findContours(
            dark_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not dark_contours or total_dark_pixels == 0:
            return None

        largest_dark_contour = max(dark_contours, key=cv2.contourArea)
        largest_area = cv2.contourArea(largest_dark_contour)
        blob_ratio = largest_area / float(total_dark_pixels)
        if blob_ratio <= self._blob_ratio_threshold:
            return None

        if not self._is_centered_blob(largest_dark_contour, crop.shape):
            return None

        return BugCandidate(
            bbox=(x, y, width, height),
            contour_area=float(cv2.contourArea(contour)),
            dark_pixel_ratio=dark_pixel_ratio,
            blob_ratio=blob_ratio,
        )

    def _is_centered_blob(
        self,
        contour: np.ndarray,
        crop_shape: tuple[int, int],
    ) -> bool:
        moments = cv2.moments(contour)
        if moments["m00"] == 0.0:
            return False

        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]
        height, width = crop_shape
        return (
            0.2 * width <= center_x <= 0.8 * width
            and 0.2 * height <= center_y <= 0.8 * height
        )

    def _publish_results(self, count: int) -> None:
        detected = count > 0

        detected_msg = Bool()
        detected_msg.data = detected
        self._bug_detected_publisher.publish(detected_msg)

        count_msg = Int32()
        count_msg.data = count
        self._bug_count_publisher.publish(count_msg)

        debug_msg = String()
        debug_msg.data = (
            f"bug_detected={detected}; "
            f"bed_id={self._current_bed_id}; "
            f"count={count}"
        )
        self._debug_publisher.publish(debug_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BugDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

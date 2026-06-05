from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Int32, String


class BugDetectionSiftNode(Node):
    """Detect the bug tag in wrist camera frames using SIFT template matching."""

    def __init__(self) -> None:
        super().__init__("bug_detection_node")
        self.declare_parameter("camera_topic", "/gripper_camera/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("template_path", "models/bug_template.png")
        self.declare_parameter("min_matches", 10)
        self.declare_parameter("ratio_threshold", 0.75)

        self._camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        self._use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )
        self._template_path = (
            self.get_parameter("template_path").get_parameter_value().string_value
        )
        self._min_matches = (
            self.get_parameter("min_matches").get_parameter_value().integer_value
        )
        self._ratio_threshold = (
            self.get_parameter("ratio_threshold").get_parameter_value().double_value
        )

        self._subscribed_camera_topic = self._camera_topic
        if self._use_compressed:
            self._subscribed_camera_topic = f"{self._camera_topic}/compressed"

        self._bridge = CvBridge()
        self._sift = cv2.SIFT_create()
        self._matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        self._current_bed_id = -1
        self._template = self._load_template()
        self._template_keypoints = []
        self._template_descriptors = None
        if self._template is not None:
            (
                self._template_keypoints,
                self._template_descriptors,
            ) = self._sift.detectAndCompute(self._template, None)

        self._bug_detected_publisher = self.create_publisher(
            Bool,
            "/simbiosys/bug_detected",
            10,
        )
        self._debug_publisher = self.create_publisher(
            String,
            "/simbiosys/bug_debug",
            10,
        )
        self._debug_image_publisher = self.create_publisher(
            Image,
            "/simbiosys/bug_debug_image",
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
            f"SIFT bug detection listening on {self._subscribed_camera_topic}, "
            "publishing /simbiosys/bug_detected, /simbiosys/bug_debug, "
            "and /simbiosys/bug_debug_image"
        )
        self.get_logger().info(
            f"SIFT template_path={self._resolve_template_path()}, "
            f"min_matches={self._min_matches}, "
            f"ratio_threshold={self._ratio_threshold:.2f}"
        )

    def _resolve_template_path(self) -> Path:
        template_path = Path(self._template_path).expanduser()
        if not template_path.is_absolute():
            template_path = Path(__file__).parent / template_path
        return template_path

    def _load_template(self) -> np.ndarray | None:
        template_path = self._resolve_template_path()
        template = cv2.imread(str(template_path), cv2.IMREAD_GRAYSCALE)
        if template is None:
            self.get_logger().warning(f"Could not load SIFT bug template: {template_path}")
        return template

    def _on_current_bed_id(self, msg: Int32) -> None:
        self._current_bed_id = msg.data

    def _on_image(self, image_msg: Image | CompressedImage) -> None:
        frame = self._image_msg_to_frame(image_msg)
        if frame is None:
            return

        detected, good_matches, debug_frame = self._detect_bug(frame)

        self._publish_results(detected, len(good_matches), debug_frame)
        if detected:
            self.get_logger().info(
                f"Bug detected! matches={len(good_matches)}, "
                f"bed_id={self._current_bed_id}"
            )
        else:
            self.get_logger().info(
                f"No bug detected. matches={len(good_matches)}, "
                f"bed_id={self._current_bed_id}"
            )

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

    def _detect_bug(
        self,
        frame: np.ndarray,
    ) -> tuple[bool, list[cv2.DMatch], np.ndarray]:
        debug_frame = frame.copy()
        if self._template is None or self._template_descriptors is None:
            return False, [], debug_frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_keypoints, image_descriptors = self._sift.detectAndCompute(gray, None)
        good_matches = self._match_descriptors(image_descriptors)
        detected = len(good_matches) >= self._min_matches

        if detected:
            self._draw_bug_bounds(debug_frame, image_keypoints, good_matches)

        return detected, good_matches, debug_frame

    def _match_descriptors(self, image_descriptors) -> list[cv2.DMatch]:
        if image_descriptors is None:
            return []

        matches = self._matcher.knnMatch(
            self._template_descriptors,
            image_descriptors,
            k=2,
        )
        good_matches = []
        for match_pair in matches:
            if len(match_pair) != 2:
                continue

            first, second = match_pair
            if first.distance < self._ratio_threshold * second.distance:
                good_matches.append(first)

        return good_matches

    def _draw_bug_bounds(
        self,
        debug_frame: np.ndarray,
        image_keypoints,
        good_matches: list[cv2.DMatch],
    ) -> None:
        if len(good_matches) < 4:
            return

        source_points = np.float32(
            [self._template_keypoints[match.queryIdx].pt for match in good_matches]
        ).reshape(-1, 1, 2)
        destination_points = np.float32(
            [image_keypoints[match.trainIdx].pt for match in good_matches]
        ).reshape(-1, 1, 2)
        homography, _ = cv2.findHomography(
            source_points,
            destination_points,
            cv2.RANSAC,
            5.0,
        )
        if homography is None:
            return

        height, width = self._template.shape
        corners = np.float32(
            [[0, 0], [width, 0], [width, height], [0, height]]
        ).reshape(-1, 1, 2)
        transformed_corners = cv2.perspectiveTransform(corners, homography)
        cv2.polylines(
            debug_frame,
            [np.int32(transformed_corners)],
            True,
            (0, 0, 255),
            3,
        )

    def _publish_results(
        self,
        detected: bool,
        matches_count: int,
        debug_frame: np.ndarray,
    ) -> None:
        detected_msg = Bool()
        detected_msg.data = detected
        self._bug_detected_publisher.publish(detected_msg)

        debug_msg = String()
        debug_msg.data = (
            f"bug_detected={detected}; "
            f"matches={matches_count}; "
            f"bed_id={self._current_bed_id}"
        )
        self._debug_publisher.publish(debug_msg)

        try:
            debug_image_msg = self._bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self._debug_image_publisher.publish(debug_image_msg)
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not publish debug image: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BugDetectionSiftNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

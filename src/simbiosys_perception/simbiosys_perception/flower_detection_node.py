from dataclasses import dataclass

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

import cv2
import numpy as np
from simbiosys_interfaces.msg import FlowerData


@dataclass(frozen=True)
class FlowerDetection:
    bbox: tuple[int, int, int, int]
    top_pixel: tuple[int, int]


class FlowerDetectionNode(Node):
    """Detect tulip flowers and estimate their height from aligned depth."""

    def __init__(self) -> None:
        super().__init__("flower_detection_node")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "simbiosys/flower_data")
        self.declare_parameter("min_contour_area", 500.0)
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("depth_roi_radius_px", 4)
        self.declare_parameter("focal_length_y_px", 615.0)

        self._image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self._depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self._depth_camera_info_topic = (
            self.get_parameter("depth_camera_info_topic")
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self._min_contour_area = (
            self.get_parameter("min_contour_area").get_parameter_value().double_value
        )
        self._morph_kernel_size = (
            self.get_parameter("morph_kernel_size").get_parameter_value().integer_value
        )
        self._depth_unit_scale = (
            self.get_parameter("depth_unit_scale").get_parameter_value().double_value
        )
        self._depth_roi_radius_px = (
            self.get_parameter("depth_roi_radius_px")
            .get_parameter_value()
            .integer_value
        )
        self._focal_length_y_px = (
            self.get_parameter("focal_length_y_px").get_parameter_value().double_value
        )

        self._bridge = CvBridge()
        self._latest_depth_m: np.ndarray | None = None
        self._principal_y_px: float | None = None
        self._publisher = self.create_publisher(FlowerData, output_topic, 10)
        self._image_subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            10,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            self._depth_topic,
            self._on_depth,
            10,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._depth_camera_info_topic,
            self._on_camera_info,
            10,
        )

        self.get_logger().info(
            f"Flower detection listening on color={self._image_topic}, "
            f"depth={self._depth_topic}, publishing {output_topic}"
        )

    def _on_camera_info(self, camera_info_msg: CameraInfo) -> None:
        focal_length_y_px = camera_info_msg.k[4]
        if focal_length_y_px > 0.0:
            self._focal_length_y_px = focal_length_y_px
            self._principal_y_px = camera_info_msg.k[5]

    def _on_depth(self, depth_msg: Image) -> None:
        try:
            depth_image = self._bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert depth image: {exc}")
            return

        self._latest_depth_m = self._depth_image_to_meters(
            depth_image,
            depth_msg.encoding,
        )

    def _on_image(self, image_msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert camera image: {exc}")
            return

        detection = self._detect_flower(frame)
        height_cm = self._estimate_height_cm(detection, frame.shape)
        msg = self._build_message(detection, height_cm, frame.shape)
        self._publisher.publish(msg)

    def _detect_flower(self, frame: np.ndarray) -> FlowerDetection | None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red tulips wrap around HSV hue 0, so use two ranges and combine them.
        lower_red_1 = np.array([0, 80, 50])
        upper_red_1 = np.array([12, 255, 255])
        lower_red_2 = np.array([165, 80, 50])
        upper_red_2 = np.array([179, 255, 255])

        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask_1, mask_2)

        kernel_size = max(1, self._morph_kernel_size)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area < self._min_contour_area:
            return None

        x, y, width, height = cv2.boundingRect(largest_contour)
        points = largest_contour.reshape(-1, 2)
        top_y = int(points[:, 1].min())
        top_candidates = points[points[:, 1] == top_y]
        top_x = int(np.median(top_candidates[:, 0]))
        return FlowerDetection(
            bbox=(x, y, width, height),
            top_pixel=(top_x, top_y),
        )

    def _depth_image_to_meters(
        self,
        depth_image: np.ndarray,
        encoding: str,
    ) -> np.ndarray:
        is_integer_depth = np.issubdtype(depth_image.dtype, np.integer)
        if encoding in ("16UC1", "mono16") or is_integer_depth:
            return depth_image.astype(np.float32) * self._depth_unit_scale

        return depth_image.astype(np.float32)

    def _estimate_height_cm(
        self,
        detection: FlowerDetection | None,
        color_shape: tuple[int, int, int],
    ) -> float | None:
        if detection is None or self._latest_depth_m is None:
            return None
        if self._focal_length_y_px <= 0.0:
            self.get_logger().warning("focal_length_y_px must be greater than zero")
            return None

        depth = self._latest_depth_m
        depth_height, depth_width = depth.shape[:2]
        color_height, color_width = color_shape[:2]
        scale_x = depth_width / color_width
        scale_y = depth_height / color_height

        x, y, width, height = detection.bbox
        top_x, top_y = detection.top_pixel
        ground_x = x + width / 2.0
        ground_y = y + height - 1

        top_depth_x = int(round(top_x * scale_x))
        top_depth_y = int(round(top_y * scale_y))
        ground_depth_x = int(round(ground_x * scale_x))
        ground_depth_y = int(round(ground_y * scale_y))

        top_depth_m = self._sample_depth_m(depth, top_depth_x, top_depth_y)
        ground_depth_m = self._sample_depth_m(depth, ground_depth_x, ground_depth_y)
        if top_depth_m is None or ground_depth_m is None:
            return None

        principal_y = self._principal_y_px
        if principal_y is None:
            principal_y = depth_height / 2.0

        top_y_m = (top_depth_y - principal_y) * top_depth_m / self._focal_length_y_px
        ground_y_m = (
            (ground_depth_y - principal_y)
            * ground_depth_m
            / self._focal_length_y_px
        )
        return abs(ground_y_m - top_y_m) * 100.0

    def _sample_depth_m(
        self,
        depth: np.ndarray,
        x: int,
        y: int,
    ) -> float | None:
        radius = max(1, self._depth_roi_radius_px)
        height, width = depth.shape[:2]
        x_min = max(0, x - radius)
        x_max = min(width, x + radius + 1)
        y_min = max(0, y - radius)
        y_max = min(height, y + radius + 1)

        roi = depth[y_min:y_max, x_min:x_max]
        valid_depths = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 10.0)]
        if valid_depths.size == 0:
            return None

        return float(np.median(valid_depths))

    def _build_message(
        self,
        detection: FlowerDetection | None,
        height_cm: float | None,
        frame_shape: tuple[int, int, int],
    ) -> FlowerData:
        msg = FlowerData()
        msg.label = "tulip"

        if detection is None:
            msg.detected = False
            msg.confidence = 0.0
            msg.position = Point()
            msg.message = "No tulip flower detected"
            return msg

        x, y, width, height = detection.bbox
        frame_height, frame_width = frame_shape[:2]
        center_x = x + width / 2.0
        center_y = y + height / 2.0
        bbox_area = float(width * height)
        frame_area = float(frame_width * frame_height)

        msg.detected = True
        msg.confidence = min(1.0, bbox_area / frame_area * 20.0)
        msg.position = Point(x=center_x, y=center_y, z=height_cm or 0.0)
        height_text = (
            f"height_cm={height_cm:.1f}"
            if height_cm is not None
            else "height_cm=unknown"
        )
        msg.message = (
            f"bbox x={x} y={y} width={width} height={height}; "
            f"top_pixel x={detection.top_pixel[0]} y={detection.top_pixel[1]}; "
            f"{height_text}"
        )
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlowerDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

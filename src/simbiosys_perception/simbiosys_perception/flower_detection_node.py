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
    color_label: str
    contour_area: float


class FlowerDetectionNode(Node):
    """Detect Dahlia flowers and estimate their height from aligned depth."""

    def __init__(self) -> None:
        super().__init__("flower_detection_node")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "simbiosys/flower_data")
        self.declare_parameter("min_contour_area", 5000.0)
        self.declare_parameter("morph_kernel_size", 7)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("depth_roi_radius_px", 4)
        self.declare_parameter("focal_length_y_px", 615.0)
        self.declare_parameter("camera_height_mm", 80.0)
        self.declare_parameter("box_height_mm", 190.0)
        self.declare_parameter("camera_distance_mm", 300.0)
        self.declare_parameter("magenta_hsv_lower", [145, 100, 80])
        self.declare_parameter("magenta_hsv_upper", [172, 255, 255])
        self.declare_parameter("light_pink_hsv_lower", [0, 15, 160])
        self.declare_parameter("light_pink_hsv_upper", [20, 60, 255])
        self.declare_parameter("white_hsv_lower", [20, 25, 180])
        self.declare_parameter("white_hsv_upper", [35, 140, 255])

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
        self._camera_height_mm = (
            self.get_parameter("camera_height_mm").get_parameter_value().double_value
        )
        self._box_height_mm = (
            self.get_parameter("box_height_mm").get_parameter_value().double_value
        )
        self._camera_distance_mm = (
            self.get_parameter("camera_distance_mm").get_parameter_value().double_value
        )

        self._bridge = CvBridge()
        self._latest_depth_m: np.ndarray | None = None
        self._principal_y_px: float | None = None
        self._publisher = self.create_publisher(FlowerData, output_topic, 10)
        self._debug_image_publisher = self.create_publisher(
            Image,
            "/simbiosys/flower_debug_image",
            10,
        )
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
        self.get_logger().info(
            f"Flower height geometry camera_height={self._camera_height_mm:.1f}mm, "
            f"box_height={self._box_height_mm:.1f}mm, "
            f"camera_distance={self._camera_distance_mm:.1f}mm"
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
        self._publish_debug_image(frame, detection, height_cm, image_msg)

    def _detect_flower(self, frame: np.ndarray) -> FlowerDetection | None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        best_detection: FlowerDetection | None = None
        for color_label, mask in self._build_color_masks(hsv):
            detection = self._find_best_detection_for_mask(mask, hsv, color_label)
            if detection is None:
                continue
            if (
                best_detection is None
                or detection.contour_area > best_detection.contour_area
            ):
                best_detection = detection

        return best_detection

    def _build_color_masks(
        self,
        hsv: np.ndarray,
    ) -> list[tuple[str, np.ndarray]]:
        masks = []
        for color_label in ("magenta", "light_pink", "white"):
            lower, upper = self._get_hsv_bounds(color_label)
            mask = cv2.inRange(hsv, lower, upper)
            masks.append((color_label, self._clean_mask(mask)))

        return masks

    def _get_hsv_bounds(self, color_label: str) -> tuple[np.ndarray, np.ndarray]:
        lower = self._get_hsv_parameter(f"{color_label}_hsv_lower")
        upper = self._get_hsv_parameter(f"{color_label}_hsv_upper")
        return np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)

    def _get_hsv_parameter(self, name: str) -> list[int]:
        values = self.get_parameter(name).get_parameter_value().integer_array_value
        if len(values) != 3:
            self.get_logger().warning(
                f"{name} must contain exactly 3 integers; using [0, 0, 0]"
            )
            return [0, 0, 0]

        return [
            max(0, min(179, int(values[0]))),
            max(0, min(255, int(values[1]))),
            max(0, min(255, int(values[2]))),
        ]

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel_size = max(1, self._morph_kernel_size)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _find_best_detection_for_mask(
        self,
        mask: np.ndarray,
        _hsv: np.ndarray,
        color_label: str,
    ) -> FlowerDetection | None:
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        valid_contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour) > self._min_contour_area
        ]
        if not valid_contours:
            return None

        combined_points = np.vstack(valid_contours)
        combined_area = float(
            sum(cv2.contourArea(contour) for contour in valid_contours)
        )
        return self._detection_from_combined_contours(
            combined_points,
            color_label,
            combined_area,
        )

    def _detection_from_combined_contours(
        self,
        combined_points: np.ndarray,
        color_label: str,
        contour_area: float,
    ) -> FlowerDetection | None:
        x, y, width, height = cv2.boundingRect(combined_points)
        aspect_ratio = width / float(height)
        if aspect_ratio < 0.4 or aspect_ratio > 2.5:
            return None

        points = combined_points.reshape(-1, 2)
        top_y = int(points[:, 1].min())
        top_candidates = points[points[:, 1] == top_y]
        top_x = int(np.median(top_candidates[:, 0]))
        return FlowerDetection(
            bbox=(x, y, width, height),
            top_pixel=(top_x, top_y),
            color_label=color_label,
            contour_area=contour_area,
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

        top_x, top_y = detection.top_pixel

        top_depth_x = int(round(top_x * scale_x))
        top_depth_y = int(round(top_y * scale_y))

        top_depth_m = self._sample_depth_m(depth, top_depth_x, top_depth_y)
        if top_depth_m is None:
            return None

        principal_y = self._principal_y_px
        if principal_y is None:
            principal_y = depth_height / 2.0

        # Geometry: camera is 8 cm above ground, horizontal, looking at a box
        # from 30 cm away; the box top is 19 cm above ground.
        top_y_m = (top_depth_y - principal_y) * top_depth_m / self._focal_length_y_px
        flower_top_height_above_ground_mm = self._camera_height_mm - (
            top_y_m * 1000.0
        )
        flower_height_above_box_mm = (
            flower_top_height_above_ground_mm - self._box_height_mm
        )
        return flower_height_above_box_mm / 10.0

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

        if detection is None:
            msg.detected = False
            msg.confidence = 0.0
            msg.label = "none"
            msg.position = Point()
            msg.message = "No Dahlia flower detected"
            return msg

        msg.label = detection.color_label
        x, y, width, height = detection.bbox
        frame_height, frame_width = frame_shape[:2]
        center_x = x + width / 2.0
        center_y = y + height / 2.0
        bbox_area = float(width * height)
        frame_area = float(frame_width * frame_height)
        height_value = height_cm if height_cm is not None else 0.0

        msg.detected = True
        msg.confidence = min(1.0, bbox_area / frame_area * 20.0)
        msg.position = Point(x=center_x, y=center_y, z=height_value)
        msg.message = (
            f"color={detection.color_label}; bbox=({x},{y},{width},{height}); "
            f"top=({detection.top_pixel[0]},{detection.top_pixel[1]}); "
            f"height={height_value:.1f}cm"
        )
        return msg

    def _publish_debug_image(
        self,
        frame: np.ndarray,
        detection: FlowerDetection | None,
        height_cm: float | None,
        source_msg: Image,
    ) -> None:
        debug_frame = frame.copy()
        if detection is not None:
            x, y, width, height = detection.bbox
            center = (int(x + width / 2.0), int(y + height / 2.0))
            top_pixel = detection.top_pixel
            height_value = height_cm if height_cm is not None else 0.0

            cv2.rectangle(
                debug_frame,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                2,
            )
            cv2.circle(debug_frame, top_pixel, 6, (255, 0, 0), -1)
            cv2.circle(debug_frame, center, 6, (0, 0, 255), -1)
            cv2.putText(
                debug_frame,
                detection.color_label,
                (x, max(25, y - 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                debug_frame,
                f"h={height_value:.1f}cm",
                (x, max(50, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        try:
            debug_msg = self._bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert debug image: {exc}")
            return

        debug_msg.header = source_msg.header
        self._debug_image_publisher.publish(debug_msg)


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

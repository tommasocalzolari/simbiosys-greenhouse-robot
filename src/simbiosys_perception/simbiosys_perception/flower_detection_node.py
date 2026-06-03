from dataclasses import dataclass
from pathlib import Path

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool

import cv2
import numpy as np
from simbiosys_interfaces.msg import FlowerData


@dataclass(frozen=True)
class FlowerDetection:
    bbox: tuple[int, int, int, int]
    top_pixel: tuple[int, int]
    color_label: str
    contour_area: float
    confidence: float | None = None


@dataclass
class TrackedFlowerCenter:
    x: float
    y: float
    color_label: str


class FlowerDetectionNode(Node):
    """Detect Dahlia flowers and estimate their height from aligned depth."""

    _YOLO_FLOWER_LABELS = {
        0: "magenta",
        1: "white",
        2: "light_pink",
    }
    _YOLO_BUG_CLASS_ID = 3
    _TRACKING_DISTANCE_PX = 80.0

    def __init__(self) -> None:
        super().__init__("flower_detection_node")
        default_model_path = "models/flower_model.pt"
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "simbiosys/flower_data")
        self.declare_parameter("use_yolo", True)
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("min_contour_area", 500.0)
        self.declare_parameter("max_contour_area", 50000.0)
        self.declare_parameter("morph_kernel_size", 7)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("depth_roi_radius_px", 4)
        self.declare_parameter("focal_length_y_px", 615.0)
        self.declare_parameter("camera_height_mm", 80.0)
        self.declare_parameter("box_height_mm", 190.0)
        self.declare_parameter("camera_distance_mm", 450.0)
        self.declare_parameter("magenta_hsv_lower", [165, 150, 70])
        self.declare_parameter("magenta_hsv_upper", [180, 255, 180])
        self.declare_parameter("light_hsv_lower", [0, 3, 175])
        self.declare_parameter("light_hsv_upper", [180, 90, 255])

        self._image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self._use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )
        self._subscribed_image_topic = self._image_topic
        if self._use_compressed:
            self._subscribed_image_topic = f"{self._image_topic}/compressed"
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
        self._use_yolo = self.get_parameter("use_yolo").get_parameter_value().bool_value
        self._model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self._min_contour_area = (
            self.get_parameter("min_contour_area").get_parameter_value().double_value
        )
        self._max_contour_area = (
            self.get_parameter("max_contour_area").get_parameter_value().double_value
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
        self._yolo_model = self._load_yolo_model() if self._use_yolo else None
        self._tracked_flower_centers: list[TrackedFlowerCenter] = []
        self._publisher = self.create_publisher(FlowerData, output_topic, 10)
        self._bug_detected_publisher = self.create_publisher(
            Bool,
            "/simbiosys/bug_detected",
            10,
        )
        self._debug_image_publisher = self.create_publisher(
            Image,
            "/simbiosys/flower_debug_image",
            10,
        )
        image_msg_type = CompressedImage if self._use_compressed else Image
        self._image_subscription = self.create_subscription(
            image_msg_type,
            self._subscribed_image_topic,
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
            f"Flower detection listening on color={self._subscribed_image_topic}, "
            f"depth={self._depth_topic}, publishing {output_topic}"
        )
        detection_backend = "YOLOv8" if self._yolo_model is not None else "HSV"
        self.get_logger().info(
            f"Flower detection backend={detection_backend}, model_path={self._model_path}"
        )
        self.get_logger().info(
            f"Flower height geometry camera_height={self._camera_height_mm:.1f}mm, "
            f"box_height={self._box_height_mm:.1f}mm, "
            f"camera_distance={self._camera_distance_mm:.1f}mm "
            "(intended flower scan distance)"
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

    def _on_image(self, image_msg: Image | CompressedImage) -> None:
        frame = self._image_msg_to_frame(image_msg)
        if frame is None:
            return

        detections, bug_detected = self._detect_frame(frame)
        if bug_detected:
            self._publish_bug_detected()

        height_by_detection = [
            self._estimate_height_cm(detection, frame.shape)
            for detection in detections
        ]

        if detections:
            for detection, height_cm in zip(detections, height_by_detection):
                msg = self._build_message(detection, height_cm, frame.shape)
                self._publisher.publish(msg)
        else:
            self._publisher.publish(self._build_message(None, None, frame.shape))

        self._publish_debug_image(frame, detections, height_by_detection, image_msg)

    def _load_yolo_model(self):
        model_path = Path(self._model_path).expanduser()
        if not model_path.is_absolute():
            model_path = Path(__file__).parent / model_path

        if not model_path.exists():
            self.get_logger().warning(
                f"YOLO model not found at {model_path}; falling back to HSV detection"
            )
            return None

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().warning(
                f"ultralytics is not installed ({exc}); falling back to HSV detection"
            )
            return None

        try:
            return YOLO(str(model_path))
        except Exception as exc:  # noqa: BLE001 - keep node alive if model loading fails.
            self.get_logger().warning(
                f"Could not load YOLO model from {model_path}: {exc}; "
                "falling back to HSV detection"
            )
            return None

    def _detect_frame(self, frame: np.ndarray) -> tuple[list[FlowerDetection], bool]:
        if self._use_yolo and self._yolo_model is not None:
            return self._detect_with_yolo(frame)

        return self._detect_flowers_hsv(frame), False

    def _detect_with_yolo(self, frame: np.ndarray) -> tuple[list[FlowerDetection], bool]:
        results = self._yolo_model.predict(frame, conf=0.4, iou=0.5, verbose=False)
        if not results:
            return [], False

        detections: list[FlowerDetection] = []
        bug_detected = False
        boxes = results[0].boxes
        if boxes is None:
            return detections, bug_detected

        for box in boxes:
            class_id = int(box.cls[0])
            if class_id == self._YOLO_BUG_CLASS_ID:
                bug_detected = True
                continue

            color_label = self._YOLO_FLOWER_LABELS.get(class_id)
            if color_label is None:
                continue

            x_center, y_center, box_width, box_height = box.xywh[0].cpu().numpy()
            x = max(0, int(round(x_center - box_width / 2.0)))
            y = max(0, int(round(y_center - box_height / 2.0)))
            width = max(1, int(round(box_width)))
            height = max(1, int(round(box_height)))
            confidence = float(box.conf[0]) if box.conf is not None else None
            top_pixel = (int(round(x_center)), y)
            detections.append(
                FlowerDetection(
                    bbox=(x, y, width, height),
                    top_pixel=top_pixel,
                    color_label=color_label,
                    contour_area=float(width * height),
                    confidence=confidence,
                )
            )

        detections = sorted(
            detections,
            key=lambda detection: detection.confidence
            if detection.confidence is not None
            else detection.contour_area,
            reverse=True,
        )
        self._update_tracked_flower_centers(detections)
        return detections, bug_detected

    def _publish_bug_detected(self) -> None:
        msg = Bool()
        msg.data = True
        self._bug_detected_publisher.publish(msg)

    def _update_tracked_flower_centers(
        self,
        detections: list[FlowerDetection],
    ) -> None:
        for detection in detections:
            x, y, width, height = detection.bbox
            center_x = x + width / 2.0
            center_y = y + height / 2.0

            matched_flower = None
            for tracked_flower in self._tracked_flower_centers:
                distance_px = float(
                    np.hypot(center_x - tracked_flower.x, center_y - tracked_flower.y)
                )
                if distance_px <= self._TRACKING_DISTANCE_PX:
                    matched_flower = tracked_flower
                    break

            if matched_flower is None:
                self._tracked_flower_centers.append(
                    TrackedFlowerCenter(center_x, center_y, detection.color_label)
                )
                self.get_logger().info(
                    f"New {detection.color_label} flower tracked at "
                    f"center=({center_x:.1f},{center_y:.1f}); "
                    f"total_tracked={len(self._tracked_flower_centers)}"
                )
            else:
                matched_flower.x = center_x
                matched_flower.y = center_y
                matched_flower.color_label = detection.color_label

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

    def _detect_flowers_hsv(self, frame: np.ndarray) -> list[FlowerDetection]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = []
        for color_label, mask in self._build_color_masks(hsv):
            detections.extend(self._find_detections_for_mask(mask, color_label))

        detections = sorted(
            detections,
            key=lambda detection: detection.contour_area,
            reverse=True,
        )
        self._update_tracked_flower_centers(detections)
        return detections

    def _build_color_masks(
        self,
        hsv: np.ndarray,
    ) -> list[tuple[str, np.ndarray]]:
        masks = []
        for color_label in ("magenta", "light"):
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
            max(0, min(180, int(values[0]))),
            max(0, min(255, int(values[1]))),
            max(0, min(255, int(values[2]))),
        ]

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel_size = max(1, self._morph_kernel_size)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _find_detections_for_mask(
        self,
        mask: np.ndarray,
        color_label: str,
    ) -> list[FlowerDetection]:
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._min_contour_area or area > self._max_contour_area:
                continue
            detection = self._detection_from_contour(
                contour,
                color_label,
                mask.shape[0],
                mask.shape[1],
            )
            if detection is not None:
                detections.append(detection)

        return detections

    def _detection_from_contour(
        self,
        contour: np.ndarray,
        color_label: str,
        image_height: int,
        image_width: int,
    ) -> FlowerDetection | None:
        x, y, width, height = cv2.boundingRect(contour)

        aspect_ratio = width / float(height)
        if aspect_ratio < 0.4 or aspect_ratio > 2.5:
            return None

        points = contour.reshape(-1, 2)
        top_y = int(points[:, 1].min())
        top_candidates = points[points[:, 1] == top_y]
        top_x = int(np.median(top_candidates[:, 0]))
        return FlowerDetection(
            bbox=(x, y, width, height),
            top_pixel=(top_x, top_y),
            color_label=color_label,
            contour_area=float(cv2.contourArea(contour)),
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
        if detection is None:
            return None
        if self._focal_length_y_px <= 0.0:
            self.get_logger().warning("focal_length_y_px must be greater than zero")
            return None

        color_height, color_width = color_shape[:2]
        top_x, top_y = detection.top_pixel
        depth_m = self._camera_distance_mm / 1000.0
        top_pixel_y = top_y

        if self._latest_depth_m is not None:
            depth = self._latest_depth_m
            depth_height, depth_width = depth.shape[:2]
            scale_x = depth_width / color_width
            scale_y = depth_height / color_height

            top_depth_x = int(round(top_x * scale_x))
            top_depth_y = int(round(top_y * scale_y))
            top_depth_m = self._sample_depth_m(depth, top_depth_x, top_depth_y)
            if top_depth_m is not None:
                depth_m = top_depth_m

            top_pixel_y = top_depth_y
            fallback_principal_y = depth_height / 2.0
        else:
            fallback_principal_y = color_height / 2.0

        principal_y = self._principal_y_px
        if principal_y is None:
            principal_y = fallback_principal_y

        top_y_offset_m = (
            (top_pixel_y - principal_y) * depth_m / self._focal_length_y_px
        )
        flower_top_height_above_ground_mm = self._camera_height_mm - (
            top_y_offset_m * 1000.0
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
        if detection.confidence is not None:
            msg.confidence = float(max(0.0, min(1.0, detection.confidence)))
        else:
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
        detections: list[FlowerDetection],
        height_by_detection: list[float | None],
        source_msg: Image | CompressedImage,
    ) -> None:
        debug_frame = frame.copy()
        for detection, height_cm in zip(detections, height_by_detection):
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

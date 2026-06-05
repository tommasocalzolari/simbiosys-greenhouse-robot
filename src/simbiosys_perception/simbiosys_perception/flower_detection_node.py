from dataclasses import dataclass
from pathlib import Path

import rclpy
from cv_bridge import CvBridge, CvBridgeError
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


class FlowerDetectionNode(Node):
    """Detect Dahlia flowers and estimate their height from aligned depth."""

    _YOLO_FLOWER_LABELS = {
        0: "magenta",
        1: "white",
        2: "light_pink",
    }
    _YOLO_BUG_CLASS_ID = 3

    def __init__(self) -> None:
        super().__init__("flower_detection_node")
        default_model_path = "models/flower_model (Copy).pt"
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "simbiosys/flower_data")
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("depth_roi_radius_px", 4)
        self.declare_parameter("focal_length_y_px", 615.0)
        self.declare_parameter("camera_height_mm", 80.0)
        self.declare_parameter("box_height_mm", 190.0)
        self.declare_parameter("camera_distance_mm", 450.0)

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
        self._model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
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
        self._yolo_model = self._load_yolo_model()
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
        self.get_logger().info(
            f"Flower detection backend=YOLOv8, model_path={self._model_path}"
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

        self._publisher.publish(self._build_message(detections, height_by_detection))

        self._publish_debug_image(frame, detections, height_by_detection, image_msg)

    def _load_yolo_model(self):
        model_path = Path(self._model_path).expanduser()
        if not model_path.is_absolute():
            model_path = Path(__file__).parent / model_path

        if not model_path.exists():
            self.get_logger().warning(f"YOLO model not found at {model_path}")
            return None

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().warning(f"ultralytics is not installed ({exc})")
            return None

        try:
            return YOLO(str(model_path))
        except Exception as exc:  # noqa: BLE001 - keep node alive if model loading fails.
            self.get_logger().warning(
                f"Could not load YOLO model from {model_path}: {exc}"
            )
            return None

    def _detect_frame(self, frame: np.ndarray) -> tuple[list[FlowerDetection], bool]:
        return self._detect_with_yolo(frame)

    def _detect_with_yolo(self, frame: np.ndarray) -> tuple[list[FlowerDetection], bool]:
        if self._yolo_model is None:
            return [], False

        results = self._yolo_model.predict(frame, conf=0.4, iou=0.5, verbose=False)
        if not results:
            return [], False

        image_height = frame.shape[0]
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
            if y >= image_height * 0.5:
                continue

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

        return self._apply_nms(detections), bug_detected

    def _publish_bug_detected(self) -> None:
        msg = Bool()
        msg.data = True
        self._bug_detected_publisher.publish(msg)

    def _apply_nms(
        self,
        detections: list[FlowerDetection],
    ) -> list[FlowerDetection]:
        kept: list[FlowerDetection] = []
        ordered = sorted(
            detections,
            key=lambda detection: detection.confidence
            if detection.confidence is not None
            else 0.0,
            reverse=True,
        )
        for detection in ordered:
            if all(
                self._bbox_iou(detection.bbox, kept_detection.bbox) <= 0.4
                for kept_detection in kept
            ):
                kept.append(detection)
        return kept

    def _bbox_iou(
        self,
        first: tuple[int, int, int, int],
        second: tuple[int, int, int, int],
    ) -> float:
        first_x, first_y, first_width, first_height = first
        second_x, second_y, second_width, second_height = second
        first_x2 = first_x + first_width
        first_y2 = first_y + first_height
        second_x2 = second_x + second_width
        second_y2 = second_y + second_height

        intersection_x1 = max(first_x, second_x)
        intersection_y1 = max(first_y, second_y)
        intersection_x2 = min(first_x2, second_x2)
        intersection_y2 = min(first_y2, second_y2)
        intersection_width = max(0, intersection_x2 - intersection_x1)
        intersection_height = max(0, intersection_y2 - intersection_y1)
        intersection_area = intersection_width * intersection_height
        first_area = first_width * first_height
        second_area = second_width * second_height
        union_area = first_area + second_area - intersection_area
        if union_area <= 0:
            return 0.0
        return intersection_area / union_area

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
        detections: list[FlowerDetection],
        heights_cm: list[float | None],
    ) -> FlowerData:
        msg = FlowerData()

        if not detections:
            msg.detected = False
            msg.dominant_label = "none"
            msg.dominant_count = 0
            msg.dominant_confidence = 0.0
            msg.heights_cm = []
            msg.message = "No Dahlia flower detected"
            return msg

        detections_by_label: dict[str, list[FlowerDetection]] = {}
        for detection in detections:
            detections_by_label.setdefault(detection.color_label, []).append(detection)

        dominant_label, dominant_detections = max(
            detections_by_label.items(),
            key=lambda item: (
                len(item[1]),
                self._average_confidence(item[1]),
                item[0],
            ),
        )
        ordered_heights = [
            0.0 if height_cm is None else float(height_cm)
            for detection, height_cm in sorted(
                zip(detections, heights_cm),
                key=lambda item: item[0].bbox[0],
            )
        ]

        msg.detected = True
        msg.dominant_label = dominant_label
        msg.dominant_count = len(dominant_detections)
        msg.dominant_confidence = self._average_confidence(dominant_detections)
        msg.heights_cm = ordered_heights
        msg.message = (
            f"detected={len(detections)}; dominant={msg.dominant_label}; "
            f"dominant_count={msg.dominant_count}; "
            f"dominant_confidence={msg.dominant_confidence:.2f}; "
            f"heights_cm={[round(height_cm, 1) for height_cm in ordered_heights]}"
        )
        return msg

    def _average_confidence(self, detections: list[FlowerDetection]) -> float:
        if not detections:
            return 0.0
        return float(
            sum(
                detection.confidence if detection.confidence is not None else 0.0
                for detection in detections
            )
            / len(detections)
        )

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

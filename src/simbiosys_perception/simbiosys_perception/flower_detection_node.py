from dataclasses import dataclass
from pathlib import Path
import threading
import time

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

import cv2
import numpy as np
from simbiosys_interfaces.action import AnalyzePlantScan
from simbiosys_interfaces.msg import FlowerData, PlantHealth


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
        default_model_path = "models/flower_model.pt"
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("output_topic", "simbiosys/flower_data")
        self.declare_parameter("plant_health_topic", "simbiosys/plant_health")
        self.declare_parameter(
            "analyze_plant_scan_action_name",
            "simbiosys/analyze_plant_scan",
        )
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("depth_roi_radius_px", 4)
        self.declare_parameter("bak_top_y_px", 110.0)
        self.declare_parameter("bak_bottom_y_px", 360.0)
        self.declare_parameter("box_height_mm", 200.0)

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
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        plant_health_topic = (
            self.get_parameter("plant_health_topic").get_parameter_value().string_value
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
        self._bak_top_y_px = (
            self.get_parameter("bak_top_y_px").get_parameter_value().double_value
        )
        self._bak_bottom_y_px = (
            self.get_parameter("bak_bottom_y_px").get_parameter_value().double_value
        )
        self._box_height_mm = (
            self.get_parameter("box_height_mm").get_parameter_value().double_value
        )

        self._bridge = CvBridge()
        self._latest_depth_m: np.ndarray | None = None
        self._resolved_model_path = self._resolve_model_path()
        self._model_path_exists = self._resolved_model_path.exists()
        self._yolo_model = None
        self._yolo_model_load_attempted = False
        if not self._model_path_exists:
            self.get_logger().warning(
                f"YOLO model not found at {self._resolved_model_path}"
            )
        self._callback_group = ReentrantCallbackGroup()
        self._analysis_condition = threading.Condition()
        self._analysis_seq = 0
        self._analysis_request_reserved = False
        self._active_scan_context = None
        self._active_analysis_goal_handle = None
        self._active_image_taken = False
        self._image_subscription = None
        self._latest_flower_data = FlowerData()
        self._latest_plant_health = PlantHealth()
        self._latest_analysis_message = ""
        self._publisher = self.create_publisher(FlowerData, output_topic, 10)
        self._plant_health_publisher = self.create_publisher(
            PlantHealth,
            plant_health_topic,
            10,
        )
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
        self._depth_subscription = self.create_subscription(
            Image,
            self._depth_topic,
            self._on_depth,
            10,
            callback_group=self._callback_group,
        )
        self._analyze_action_server = ActionServer(
            self,
            AnalyzePlantScan,
            self.get_parameter("analyze_plant_scan_action_name")
            .get_parameter_value()
            .string_value,
            execute_callback=self._execute_analyze_plant_scan,
            goal_callback=self._analyze_goal_callback,
            cancel_callback=self._analyze_cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"Flower detection will request color={self._subscribed_image_topic}, "
            f"depth={self._depth_topic}, publishing {output_topic}"
        )
        self.get_logger().info(
            f"Flower detection backend=YOLOv8, model_path={self._model_path}"
        )
        self.get_logger().info(
            f"Flower height mapping bak_top_y={self._bak_top_y_px:.1f}px, "
            f"bak_bottom_y={self._bak_bottom_y_px:.1f}px, "
            f"box_height={self._box_height_mm:.1f}mm"
        )

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

        with self._analysis_condition:
            if self._active_scan_context is None or self._active_image_taken:
                return
            self._active_image_taken = True
            scan_context = self._active_scan_context
            goal_handle = self._active_analysis_goal_handle

        if goal_handle is not None:
            self._publish_analyze_feedback(
                goal_handle,
                "analyzing",
                0.5,
                f"Analyzing image for {scan_context.scan_position.scan_position_id}",
        )

        detections, bug_detected = self._detect_frame(frame)
        height_by_detection = [
            self._estimate_height_cm(detection, frame.shape)
            for detection in detections
        ]

        flower_data = self._build_message(detections, height_by_detection)
        plant_health = self._plant_health_from_flower_data(
            flower_data,
            bug_detected,
            scan_context,
        )
        if not self._store_latest_analysis(
            flower_data,
            plant_health,
            scan_context,
            goal_handle,
        ):
            return

        if bug_detected:
            self._publish_bug_detected()
        self._publisher.publish(flower_data)
        self._plant_health_publisher.publish(plant_health)

        self._publish_debug_image(frame, detections, height_by_detection, image_msg)

    def _analyze_goal_callback(self, goal_request: AnalyzePlantScan.Goal) -> GoalResponse:
        if not goal_request.scan_position.scan_position_id.strip():
            self.get_logger().warning("Rejecting plant scan analysis without scan position id")
            return GoalResponse.REJECT
        with self._analysis_condition:
            if (
                self._analysis_request_reserved
                or self._active_scan_context is not None
                or self._image_subscription is not None
            ):
                self.get_logger().warning("Rejecting plant scan analysis while another request is active")
                return GoalResponse.REJECT
            self._analysis_request_reserved = True
        return GoalResponse.ACCEPT

    def _analyze_cancel_callback(self, _goal_handle) -> CancelResponse:
        self._clear_active_request()
        return CancelResponse.ACCEPT

    def _execute_analyze_plant_scan(self, goal_handle):
        goal = goal_handle.request
        result = AnalyzePlantScan.Result()

        if goal.dry_run:
            flower_data = FlowerData()
            flower_data.detected = False
            flower_data.dominant_label = "dry_run"
            flower_data.dominant_count = 0
            flower_data.dominant_confidence = 0.0
            flower_data.heights_cm = []
            flower_data.message = "DRY_RUN: plant scan analysis accepted"
            plant_health = self._plant_health_from_flower_data(
                flower_data,
                False,
                goal,
            )
            self._publisher.publish(flower_data)
            self._plant_health_publisher.publish(plant_health)
            result.success = True
            result.flower_data = flower_data
            result.plant_health = plant_health
            result.message = flower_data.message
            goal_handle.succeed()
            self._clear_active_request()
            return result

        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec > 0.0 else 5.0
        if not self._ensure_yolo_model_loaded():
            result.success = False
            result.message = (
                f"PRECONDITION_FAILED: YOLO model unavailable at "
                f"{self._resolved_model_path}"
            )
            goal_handle.abort()
            self._clear_active_request()
            return result

        with self._analysis_condition:
            start_seq = self._analysis_seq
            self._active_scan_context = goal
            self._active_analysis_goal_handle = goal_handle
            self._active_image_taken = False
            self._analysis_request_reserved = False
        self._start_image_subscription()

        self._publish_analyze_feedback(
            goal_handle,
            "waiting_for_image",
            0.1,
            f"Waiting for fresh image for {goal.scan_position.scan_position_id}",
        )

        deadline = time.monotonic() + timeout_sec
        try:
            with self._analysis_condition:
                while self._analysis_seq <= start_seq:
                    if not rclpy.ok():
                        result.success = False
                        result.message = "SHUTDOWN: plant scan analysis interrupted"
                        goal_handle.abort()
                        return result
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = "plant scan analysis cancelled"
                        return result
                    remaining_sec = deadline - time.monotonic()
                    if remaining_sec <= 0.0:
                        result.success = False
                        result.message = "TIMEOUT: no fresh image received for plant scan analysis"
                        goal_handle.abort()
                        return result
                    self._analysis_condition.wait(timeout=min(0.1, remaining_sec))

                flower_data = self._latest_flower_data
                plant_health = self._latest_plant_health
                message = self._latest_analysis_message
        finally:
            self._clear_active_request()

        self._publish_analyze_feedback(
            goal_handle,
            "completed",
            1.0,
            message or flower_data.message,
        )
        result.success = True
        result.flower_data = flower_data
        result.plant_health = plant_health
        result.message = message or flower_data.message
        goal_handle.succeed()
        return result

    def _publish_analyze_feedback(
        self,
        goal_handle,
        phase: str,
        progress: float,
        message: str,
    ) -> None:
        feedback = AnalyzePlantScan.Feedback()
        feedback.phase = phase
        feedback.progress = max(0.0, min(1.0, float(progress)))
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    def _scan_context_snapshot(self):
        with self._analysis_condition:
            return self._active_scan_context

    def _start_image_subscription(self) -> None:
        with self._analysis_condition:
            if self._image_subscription is not None:
                return
            image_msg_type = CompressedImage if self._use_compressed else Image
            self._image_subscription = self.create_subscription(
                image_msg_type,
                self._subscribed_image_topic,
                self._on_image,
                10,
                callback_group=self._callback_group,
            )

    def _release_image_subscription(self) -> None:
        with self._analysis_condition:
            subscription = self._image_subscription
            self._image_subscription = None
        if subscription is not None:
            self.destroy_subscription(subscription)

    def _clear_active_request(self) -> None:
        self._release_image_subscription()
        with self._analysis_condition:
            self._active_scan_context = None
            self._active_analysis_goal_handle = None
            self._active_image_taken = False
            self._analysis_request_reserved = False
            self._analysis_condition.notify_all()

    def _store_latest_analysis(
        self,
        flower_data: FlowerData,
        plant_health: PlantHealth,
        expected_scan_context=None,
        expected_goal_handle=None,
    ) -> bool:
        with self._analysis_condition:
            if (
                expected_scan_context is not None
                and self._active_scan_context is not expected_scan_context
            ):
                return False
            if (
                expected_goal_handle is not None
                and self._active_analysis_goal_handle is not expected_goal_handle
            ):
                return False
            self._latest_flower_data = flower_data
            self._latest_plant_health = plant_health
            self._latest_analysis_message = plant_health.notes or flower_data.message
            self._analysis_seq += 1
            self._analysis_condition.notify_all()
            return True

    def _resolve_model_path(self) -> Path:
        model_path = Path(self._model_path).expanduser()
        if model_path.is_absolute():
            return model_path
        cwd_model_path = Path.cwd() / model_path
        if cwd_model_path.exists():
            return cwd_model_path
        return Path(__file__).parent / model_path

    def _ensure_yolo_model_loaded(self) -> bool:
        if self._yolo_model is not None:
            return True
        if self._yolo_model_load_attempted:
            return False
        self._yolo_model_load_attempted = True
        self._model_path_exists = self._resolved_model_path.exists()
        if not self._model_path_exists:
            self.get_logger().warning(
                f"YOLO model not found at {self._resolved_model_path}"
            )
            return False

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().warning(f"ultralytics is not installed ({exc})")
            return None

        try:
            self._yolo_model = YOLO(str(self._resolved_model_path))
            return True
        except Exception as exc:  # noqa: BLE001 - keep node alive if model loading fails.
            self.get_logger().warning(
                f"Could not load YOLO model from {self._resolved_model_path}: {exc}"
            )
            return False

    def _detect_frame(self, frame: np.ndarray) -> tuple[list[FlowerDetection], bool]:
        return self._detect_with_yolo(frame)

    def _detect_with_yolo(self, frame: np.ndarray) -> tuple[list[FlowerDetection], bool]:
        if not self._ensure_yolo_model_loaded():
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
            if y >= image_height * 0.40:
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
        pixels_per_mm = (
            (self._bak_bottom_y_px - self._bak_top_y_px) / self._box_height_mm
        )
        if pixels_per_mm <= 0.0:
            self.get_logger().warning("Flower height mapping must be greater than zero")
            return None

        _top_x, top_pixel_y = detection.top_pixel
        pixels_above_bak = self._bak_top_y_px - top_pixel_y
        height_above_box_mm = pixels_above_bak / pixels_per_mm
        height_cm = height_above_box_mm / 10.0
        return height_cm

    def _height_warning_message(self, height_cm: float | None) -> str:
        if height_cm is None:
            return ""
        if height_cm <= 5.7:
            return "Low flower detection, probably flower detection from row behind!"
        if height_cm >= 8.8:
            return "Max flower height detected of 8.8cm so probably higher, inspect with wrist camera for accurate analysis!"
        return ""

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
        ordered_detection_heights = sorted(
            zip(detections, heights_cm),
            key=lambda item: item[0].bbox[0],
        )
        ordered_heights = [
            0.0 if height_cm is None else float(height_cm)
            for _detection, height_cm in ordered_detection_heights
        ]
        warning_messages = [
            self._height_warning_message(height_cm)
            for _detection, height_cm in ordered_detection_heights
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
        if any(warning_messages):
            msg.message += f"; warnings={[warning for warning in warning_messages if warning]}"
        return msg

    def _plant_health_from_flower_data(
        self,
        flower_data: FlowerData,
        bug_detected: bool,
        scan_context,
    ) -> PlantHealth:
        msg = PlantHealth()
        scan_position = getattr(scan_context, "scan_position", None)
        side = str(getattr(scan_context, "side", "") or "").strip().lower()
        if scan_position is not None:
            msg.bed_id = scan_position.bed_id
            msg.flower_id = self._flower_id_for_scan(scan_position, side)
            msg.position = Point(
                x=float(scan_position.base_pose.x),
                y=float(scan_position.base_pose.y),
                z=0.0,
            )
        msg.flower_detected = bool(flower_data.detected)
        msg.bug_detected = bool(bug_detected)
        msg.color = flower_data.dominant_label
        msg.confidence = float(flower_data.dominant_confidence)
        msg.height_cm = self._representative_height_cm(flower_data.heights_cm)
        msg.position.z = float(msg.height_cm)
        msg.growth_stage = "unknown"
        msg.ready_for_harvest = False
        msg.health = "unknown" if flower_data.detected else "no_detection"
        msg.last_scan_time = self.get_clock().now().to_msg()
        msg.notes = (
            f"side:{side or 'unknown'}; count:{flower_data.dominant_count}; "
            f"{flower_data.message}"
        )
        return msg

    def _flower_id_for_scan(self, scan_position, side: str) -> str:
        parts = [
            scan_position.bed_id.strip(),
            side,
            scan_position.scan_position_id.strip(),
        ]
        return ":".join(part for part in parts if part)

    def _representative_height_cm(self, heights_cm) -> float:
        valid_heights = [float(height) for height in heights_cm if float(height) > 0.0]
        if not valid_heights:
            return 0.0
        return sum(valid_heights) / len(valid_heights)

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

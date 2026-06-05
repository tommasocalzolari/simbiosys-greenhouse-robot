from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Int32, String


class BugDetectionNode(Node):
    """Detect bugs in wrist camera frames using the shared YOLOv8 model."""

    _YOLO_BUG_CLASS_ID = 3

    def __init__(self) -> None:
        super().__init__("bug_detection_node")
        default_model_path = "models/flower_model (Copy).pt"
        self.declare_parameter("camera_topic", "/gripper_camera/image_raw")
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("camera_distance_mm", 200.0)

        self._camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        self._use_compressed = (
            self.get_parameter("use_compressed").get_parameter_value().bool_value
        )
        self._subscribed_camera_topic = self._camera_topic
        if self._use_compressed:
            self._subscribed_camera_topic = f"{self._camera_topic}/compressed"
        self._model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self._camera_distance_mm = (
            self.get_parameter("camera_distance_mm").get_parameter_value().double_value
        )

        self._bridge = CvBridge()
        self._current_bed_id = -1
        self._bug_count = 0
        self._yolo_model = self._load_yolo_model()

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
        self.get_logger().info(
            f"Bug detection backend=YOLOv8, model_path={self._model_path}"
        )
        self.get_logger().info(
            f"Bug detection operating distance: {self._camera_distance_mm:.1f}mm "
            "(intended bug scan distance)"
        )

    def _on_current_bed_id(self, msg: Int32) -> None:
        self._current_bed_id = msg.data

    def _on_image(self, image_msg: Image | CompressedImage) -> None:
        frame = self._image_msg_to_frame(image_msg)
        if frame is None:
            return

        frame_bug_count = self._detect_bug_count(frame)
        self._publish_results(frame_bug_count)

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
            self.get_logger().warning(f"ultralytics is not installed: {exc}")
            return None

        try:
            return YOLO(str(model_path))
        except Exception as exc:  # noqa: BLE001 - keep node alive if loading fails.
            self.get_logger().warning(
                f"Could not load YOLO model from {model_path}: {exc}"
            )
            return None

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

    def _detect_bug_count(self, frame: np.ndarray) -> int:
        if self._yolo_model is None:
            return 0

        results = self._yolo_model.predict(frame, conf=0.4, iou=0.5, verbose=False)
        if not results or results[0].boxes is None:
            return 0

        bug_count = 0
        for box in results[0].boxes:
            if int(box.cls[0]) == self._YOLO_BUG_CLASS_ID:
                bug_count += 1

        if bug_count > 0:
            self.get_logger().info(
                f"Bug detected! frame_count={bug_count}, "
                f"bed_id={self._current_bed_id}"
            )

        return bug_count

    def _publish_results(self, frame_bug_count: int) -> None:
        detected = frame_bug_count > 0
        if detected:
            self._bug_count += frame_bug_count

        detected_msg = Bool()
        detected_msg.data = detected
        self._bug_detected_publisher.publish(detected_msg)

        count_msg = Int32()
        count_msg.data = self._bug_count
        self._bug_count_publisher.publish(count_msg)

        debug_msg = String()
        debug_msg.data = (
            f"bug_detected={detected}; "
            f"bed_id={self._current_bed_id}; "
            f"frame_count={frame_bug_count}; "
            f"total_count={self._bug_count}"
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

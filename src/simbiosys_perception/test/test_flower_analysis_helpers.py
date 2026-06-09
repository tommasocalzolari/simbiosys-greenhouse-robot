import threading

from builtin_interfaces.msg import Time
from rclpy.action import GoalResponse
from sensor_msgs.msg import CompressedImage

from simbiosys_interfaces.action import AnalyzePlantScan
from simbiosys_interfaces.msg import FlowerData, ScanPosition
from simbiosys_perception import flower_detection_node as flower_detection_module
from simbiosys_perception.flower_detection_node import FlowerDetectionNode


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        return Time(sec=12, nanosec=34)


class _ScanContext:
    def __init__(self):
        self.scan_position = ScanPosition()
        self.scan_position.scan_position_id = "bed_2_b_1"
        self.scan_position.bed_id = "bed_2"
        self.scan_position.base_pose.x = 1.2
        self.scan_position.base_pose.y = 3.4
        self.side = "b"


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _GoalHandle:
    def __init__(self, goal=None):
        self.request = goal
        self.feedback = []
        self.succeeded = False
        self.aborted = False
        self.cancelled = False
        self.is_cancel_requested = False

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True

    def canceled(self):
        self.cancelled = True


def _analysis_goal(timeout_sec=0.01):
    goal = AnalyzePlantScan.Goal()
    goal.scan_position.scan_position_id = "bed_2_b_1"
    goal.scan_position.bed_id = "bed_2"
    goal.scan_position.base_pose.x = 1.2
    goal.scan_position.base_pose.y = 3.4
    goal.side = "b"
    goal.mission_id = "mission-1"
    goal.request_id = "mission-1:bed_2_b_1:b"
    goal.timeout_sec = timeout_sec
    goal.dry_run = False
    return goal


def _lazy_node_without_ros():
    node = object.__new__(FlowerDetectionNode)
    node._analysis_condition = threading.Condition()
    node._analysis_seq = 0
    node._analysis_request_reserved = False
    node._active_scan_context = None
    node._active_analysis_goal_handle = None
    node._active_image_taken = False
    node._image_subscription = None
    node._use_compressed = True
    node._subscribed_image_topic = "/camera/color/image_raw/compressed"
    node._callback_group = object()
    node.get_logger = lambda: _Logger()
    node._ensure_yolo_model_loaded = lambda: True
    return node


def test_plant_health_uses_new_flower_data_summary_contract():
    node = object.__new__(FlowerDetectionNode)
    node.get_clock = lambda: _Clock()
    flower_data = FlowerData()
    flower_data.detected = True
    flower_data.dominant_label = "magenta"
    flower_data.dominant_count = 3
    flower_data.dominant_confidence = 0.82
    flower_data.heights_cm = [10.0, 20.0, 0.0]
    flower_data.message = "detected=3"

    plant_health = node._plant_health_from_flower_data(
        flower_data,
        True,
        _ScanContext(),
    )

    assert plant_health.flower_detected
    assert plant_health.bug_detected
    assert plant_health.bed_id == "bed_2"
    assert plant_health.flower_id == "bed_2:b:bed_2_b_1"
    assert plant_health.color == "magenta"
    assert plant_health.confidence == 0.82
    assert plant_health.height_cm == 15.0
    assert plant_health.position.x == 1.2
    assert plant_health.position.y == 3.4
    assert plant_health.position.z == 15.0


def test_analyze_goal_creates_and_releases_lazy_image_subscription():
    node = _lazy_node_without_ros()
    created = []
    destroyed = []
    subscription = object()

    def create_subscription(msg_type, topic, callback, depth, callback_group=None):
        created.append((msg_type, topic, callback, depth, callback_group))
        return subscription

    node.create_subscription = create_subscription
    node.destroy_subscription = lambda sub: destroyed.append(sub) or True

    node._start_image_subscription()

    assert node._image_subscription is subscription
    assert created == [
        (
            CompressedImage,
            "/camera/color/image_raw/compressed",
            node._on_image,
            10,
            node._callback_group,
        )
    ]

    node._release_image_subscription()

    assert node._image_subscription is None
    assert destroyed == [subscription]


def test_analyze_goal_rejects_second_active_request():
    node = _lazy_node_without_ros()
    first_goal = _analysis_goal()
    second_goal = _analysis_goal()

    assert node._analyze_goal_callback(first_goal) == GoalResponse.ACCEPT
    assert node._analyze_goal_callback(second_goal) == GoalResponse.REJECT


def test_relative_model_path_prefers_workspace_cwd(tmp_path, monkeypatch):
    model_path = tmp_path / "models" / "flower_model.pt"
    model_path.parent.mkdir()
    model_path.write_bytes(b"model")
    monkeypatch.chdir(tmp_path)
    node = object.__new__(FlowerDetectionNode)
    node._model_path = "models/flower_model.pt"

    assert node._resolve_model_path() == model_path


def test_analyze_timeout_releases_lazy_subscription(monkeypatch):
    monkeypatch.setattr(flower_detection_module.rclpy, "ok", lambda: True)
    node = _lazy_node_without_ros()
    subscription = object()
    destroyed = []
    node.create_subscription = lambda *args, **kwargs: subscription
    node.destroy_subscription = lambda sub: destroyed.append(sub) or True

    goal_handle = _GoalHandle(_analysis_goal(timeout_sec=0.001))

    result = node._execute_analyze_plant_scan(goal_handle)

    assert not result.success
    assert goal_handle.aborted
    assert "TIMEOUT" in result.message
    assert node._image_subscription is None
    assert destroyed == [subscription]
    assert node._active_scan_context is None
    assert node._active_analysis_goal_handle is None


def test_cancel_releases_lazy_subscription_and_clears_active_request():
    node = _lazy_node_without_ros()
    subscription = object()
    destroyed = []
    node._image_subscription = subscription
    node._analysis_request_reserved = True
    node._active_scan_context = _analysis_goal()
    node._active_analysis_goal_handle = _GoalHandle()
    node._active_image_taken = True
    node.destroy_subscription = lambda sub: destroyed.append(sub) or True

    node._analyze_cancel_callback(node._active_analysis_goal_handle)

    assert node._image_subscription is None
    assert destroyed == [subscription]
    assert node._active_scan_context is None
    assert node._active_analysis_goal_handle is None
    assert not node._active_image_taken
    assert not node._analysis_request_reserved


def test_stale_image_result_is_not_published_after_cancel():
    node = _lazy_node_without_ros()
    goal = _analysis_goal()
    goal_handle = _GoalHandle(goal)
    node._active_scan_context = goal
    node._active_analysis_goal_handle = goal_handle
    node._publisher = _Publisher()
    node._plant_health_publisher = _Publisher()
    node._bug_detected_publisher = _Publisher()
    node._debug_image_publisher = _Publisher()
    node._latest_depth_m = None
    node._focal_length_y_px = 615.0
    node._principal_y_px = None
    node._camera_distance_mm = 450.0
    node._camera_height_mm = 80.0
    node._box_height_mm = 190.0
    node._depth_roi_radius_px = 4
    node._image_msg_to_frame = lambda _msg: object()
    node._detect_frame = lambda _frame: ([], True)
    node._build_message = lambda _detections, _heights: FlowerData()
    node._plant_health_from_flower_data = lambda _flower, _bug, _context: object()
    node._estimate_height_cm = lambda _detection, _shape: 0.0
    node._publish_debug_image = lambda *_args: None

    def clear_before_store(_flower_data, _plant_health, _context, _goal_handle):
        node._active_scan_context = None
        return FlowerDetectionNode._store_latest_analysis(
            node,
            _flower_data,
            _plant_health,
            _context,
            _goal_handle,
        )

    node._store_latest_analysis = clear_before_store

    node._on_image(CompressedImage())

    assert node._publisher.messages == []
    assert node._plant_health_publisher.messages == []
    assert node._bug_detected_publisher.messages == []

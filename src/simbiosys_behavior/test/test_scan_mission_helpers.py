import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist

from simbiosys_behavior.mission_manager_node import (
    MissionManagerNode,
    QueuedScanPosition,
)
from simbiosys_behavior.scan_pose_controller_node import ScanPoseControllerNode
from simbiosys_interfaces.msg import BedSideAlignment, PlantHealth, ScanPosition


class _Logger:
    def warning(self, _message):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _mission_manager_without_ros():
    node = object.__new__(MissionManagerNode)
    node.get_logger = lambda: _Logger()
    node.get_clock = lambda: type("Clock", (), {"now": lambda _self: type("Now", (), {"to_msg": lambda _now: Time()})()})()
    return node


def _scan_position(scan_id, bed_id, side, order=0):
    position = ScanPosition()
    position.scan_position_id = scan_id
    position.bed_id = bed_id
    position.base_pose.x = 1.0
    position.base_pose.y = 2.0
    position.base_pose.theta = 0.5
    position.order = order
    position.enabled = True
    return QueuedScanPosition(position, side)


def test_parse_scan_position_csv_entry():
    node = _mission_manager_without_ros()

    parsed = node._parse_scan_position_entry("bed_1_a_1,bed_1,a,1.2,2.3,0.4", 3)

    assert parsed is not None
    assert parsed.scan_position.scan_position_id == "bed_1_a_1"
    assert parsed.scan_position.bed_id == "bed_1"
    assert parsed.side == "a"
    assert parsed.scan_position.base_pose.x == 1.2
    assert parsed.scan_position.base_pose.y == 2.3
    assert parsed.scan_position.base_pose.theta == 0.4
    assert parsed.scan_position.order == 3


def test_parse_scan_position_json_entry():
    node = _mission_manager_without_ros()

    parsed = node._parse_scan_position_entry(
        '{"id":"bed_2_b_2","bed_id":"bed_2","side":"b",'
        '"x":3.0,"y":4.0,"yaw":1.57,"order":7}',
        0,
    )

    assert parsed is not None
    assert parsed.scan_position.scan_position_id == "bed_2_b_2"
    assert parsed.scan_position.bed_id == "bed_2"
    assert parsed.side == "b"
    assert parsed.scan_position.order == 7


def test_scan_queue_filters_bed_side_target():
    node = _mission_manager_without_ros()
    node._parse_scan_positions = lambda: [
        _scan_position("bed_1_a_1", "bed_1", "a", 0),
        _scan_position("bed_1_b_1", "bed_1", "b", 1),
        _scan_position("bed_2_a_1", "bed_2", "a", 2),
    ]

    filtered = node._scan_queue_for_target("bed_1:b")

    assert [entry.scan_position.scan_position_id for entry in filtered] == [
        "bed_1_b_1"
    ]


def test_checkpoint_payload_builds_scan_target():
    node = _mission_manager_without_ros()
    payload = {
        "label": "checkpoint_1",
        "metadata": {
            "bed_id": "1",
            "side": "a",
            "scan_position_id": "bed_1_a_1",
            "order": 1,
            "target_distance_m": 0.42,
        },
        "pose": {
            "frame_id": "map",
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }

    target = node._target_from_checkpoint_payload(payload)

    assert target is not None
    assert target.scan_position.scan_position_id == "bed_1_a_1"
    assert target.scan_position.bed_id == "1"
    assert target.side == "a"
    assert target.scan_position.base_pose.x == 1.0
    assert target.scan_position.base_pose.y == 2.0
    assert target.target_distance_m == 0.42


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _WrappedResult:
    def __init__(self, result):
        self.result = result


class _PlantAnalysisGoalHandle:
    accepted = True

    def __init__(self, result):
        self._result = result

    def get_result_async(self):
        return _Future(_WrappedResult(self._result))


class _PlantAnalysisClient:
    def __init__(self, result):
        self.result = result
        self.sent_goal = None

    def wait_for_server(self, timeout_sec):
        return timeout_sec == 0.5

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent_goal = goal
        self.feedback_callback = feedback_callback
        return _Future(_PlantAnalysisGoalHandle(self.result))


class _BehaviorGoalHandle:
    is_cancel_requested = False


def test_mission_manager_requests_plant_analysis_action():
    node = _mission_manager_without_ros()
    node._params = {
        "plant_analysis_server_timeout_sec": 0.5,
        "plant_analysis_timeout_sec": 4.0,
        "plant_analysis_dry_run": False,
    }
    node._double_parameter = lambda name: float(node._params[name])
    node._bool_parameter = lambda name: bool(node._params[name])
    node._cancel_active_work = lambda: None
    node._latest_plant_health = PlantHealth()
    node._latest_plant_health_time = 0.0
    result = type("Result", (), {})()
    result.success = True
    result.message = "fresh analysis"
    result.plant_health = PlantHealth()
    client = _PlantAnalysisClient(result)
    node._plant_analysis_client = client
    queued_position = _scan_position("bed_1_a_1", "bed_1", "a")

    success, message = node._execute_plant_analysis(
        _BehaviorGoalHandle(),
        "mission-1",
        queued_position,
    )

    assert success
    assert message == "fresh analysis"
    assert client.sent_goal.scan_position.scan_position_id == "bed_1_a_1"
    assert client.sent_goal.side == "a"
    assert client.sent_goal.mission_id == "mission-1"
    assert client.sent_goal.request_id == "mission-1:bed_1_a_1:a"
    assert client.sent_goal.timeout_sec == 4.0


def _scan_controller_without_ros(params=None):
    node = object.__new__(ScanPoseControllerNode)
    node._params = {
        "alignment_timeout_sec": 0.5,
        "min_confidence": 0.25,
        "distance_tolerance_m": 0.01,
        "yaw_tolerance_rad": math.radians(1.0),
        "distance_gain": 1.0,
        "yaw_gain": 3.0,
        "max_forward_speed_mps": 0.5,
        "max_angular_speed_radps": 2.0,
        "min_forward_speed_mps": 0.0,
        "min_angular_speed_radps": 0.0,
        "alignment_filter_alpha": 1.0,
    }
    if params:
        node._params.update(params)
    node._double_parameter = lambda name: node._params[name]
    node._cmd_vel_publisher = _Publisher()
    node._filtered_distance_error = None
    node._filtered_yaw_error = None
    node._last_control_twist = None
    node._last_control_twist_time = 0.0
    return node


def _alignment(distance_error=0.0, yaw_error=0.0, confidence=1.0, valid=True):
    msg = BedSideAlignment()
    msg.valid = valid
    msg.distance_m = 0.35 + distance_error
    msg.target_distance_m = 0.35
    msg.distance_error_m = distance_error
    msg.yaw_error_rad = yaw_error
    msg.confidence = confidence
    return msg


def test_scan_pose_alignment_requires_fresh_confident_valid_message():
    node = _scan_controller_without_ros()

    assert node._alignment_is_usable(_alignment(), 0.1)
    assert not node._alignment_is_usable(_alignment(valid=False), 0.1)
    assert not node._alignment_is_usable(_alignment(confidence=0.1), 0.1)
    assert not node._alignment_is_usable(_alignment(), 1.0)


def test_scan_pose_control_stops_when_aligned():
    node = _scan_controller_without_ros()

    aligned = node._publish_control_twist(_alignment(), 0.35)

    assert aligned
    assert isinstance(node._cmd_vel_publisher.messages[-1], Twist)
    assert node._cmd_vel_publisher.messages[-1].linear.x == 0.0
    assert node._cmd_vel_publisher.messages[-1].angular.z == 0.0


def test_scan_pose_control_commands_distance_and_yaw_correction():
    node = _scan_controller_without_ros()

    aligned = node._publish_control_twist(
        _alignment(distance_error=0.2, yaw_error=0.2),
        0.35,
    )

    assert not aligned
    twist = node._cmd_vel_publisher.messages[-1]
    assert twist.linear.x == 0.2
    assert twist.angular.z == pytest.approx(0.6)

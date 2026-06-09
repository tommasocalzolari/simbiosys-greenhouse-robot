import threading

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.action import GoalResponse

from simbiosys_behavior.simple_mission_manager_node import (
    MissionTarget,
    SimpleMissionManagerNode,
)
from simbiosys_interfaces.msg import BehaviorType, ScanPosition


class _Logger:
    def warning(self, _message):
        pass


def _node_without_ros():
    node = object.__new__(SimpleMissionManagerNode)
    node._mission_lock = threading.Lock()
    node._mission_reserved = False
    node.get_logger = lambda: _Logger()
    node.get_clock = lambda: type(
        "Clock",
        (),
        {
            "now": lambda _self: type(
                "Now",
                (),
                {"to_msg": lambda _now: Time()},
            )()
        },
    )()
    return node


def _request(behavior_type):
    behavior = type("Behavior", (), {"type": behavior_type})()
    return type("Request", (), {"behavior": behavior})()


def _target():
    scan_position = ScanPosition()
    scan_position.scan_position_id = "bed_1_a_1"
    scan_position.bed_id = "1"
    return MissionTarget(
        label="checkpoint_1",
        scan_position=scan_position,
        side="a",
        pose=PoseStamped(),
        target_distance_m=0.35,
    )


def test_only_one_checkpoint_mission_is_admitted():
    node = _node_without_ros()

    first = node._goal_callback(_request(BehaviorType.INSPECT_BED))
    second = node._goal_callback(_request(BehaviorType.INSPECT_BED))

    assert first == GoalResponse.ACCEPT
    assert second == GoalResponse.REJECT


def test_idle_remains_available_while_mission_is_reserved():
    node = _node_without_ros()
    node._mission_reserved = True

    response = node._goal_callback(_request(BehaviorType.IDLE))

    assert response == GoalResponse.ACCEPT


def test_target_filter_accepts_all_bed_side_and_checkpoint():
    node = _node_without_ros()
    target = _target()

    assert node._target_matches("all", target)
    assert node._target_matches("1", target)
    assert node._target_matches("1:a", target)
    assert node._target_matches("checkpoint_1", target)
    assert node._target_matches("bed_1_a_1", target)
    assert not node._target_matches("2", target)


def test_checkpoint_annotation_is_converted_to_scan_target():
    node = _node_without_ros()
    annotation = {
        "label": "checkpoint_1",
        "bed_id": "1",
        "side": "a",
        "scan_position_id": "bed_1_a_1",
        "order": 1,
        "target_distance_m": 0.42,
        "pose": {
            "frame_id": "map",
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }

    target = node._target_from_annotation(annotation)

    assert target is not None
    assert target.label == "checkpoint_1"
    assert target.scan_position.scan_position_id == "bed_1_a_1"
    assert target.scan_position.bed_id == "1"
    assert target.side == "a"
    assert target.target_distance_m == 0.42
    assert target.scan_position.base_pose.theta == 0.0

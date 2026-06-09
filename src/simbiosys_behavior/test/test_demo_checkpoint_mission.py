from simbiosys_behavior.demo_checkpoint_mission_node import (
    DemoCheckpointMissionNode,
)


def _node_without_ros():
    node = object.__new__(DemoCheckpointMissionNode)
    values = {
        "checkpoints_per_bed": 4,
        "checkpoints_per_side": 2,
    }
    node._int_parameter = lambda name: values[name]
    return node


def test_scan_target_uses_checkpoint_metadata_and_pose():
    node = _node_without_ros()
    target = {
        "label": "checkpoint_2",
        "metadata": {
            "bed_id": "1",
            "side": "a",
            "lane": "right",
            "scan_position_id": "bed_1_a_2",
            "order": 2,
        },
        "pose": {
            "position": {"x": 1.25, "y": -0.5, "z": 0.0},
        },
    }

    scan_position, side, lane = node._scan_target(target)

    assert scan_position.scan_position_id == "bed_1_a_2"
    assert scan_position.bed_id == "1"
    assert scan_position.order == 2
    assert scan_position.base_pose.x == 1.25
    assert scan_position.base_pose.y == -0.5
    assert side == "a"
    assert lane == "right"


def test_scan_target_rejects_missing_required_metadata():
    node = _node_without_ros()

    scan_position, side, lane = node._scan_target(
        {"label": "", "metadata": {"side": "b"}, "pose": {}}
    )

    assert scan_position is None
    assert side == "b"
    assert lane == ""


def test_scan_target_derives_metadata_for_pose_only_checkpoint():
    node = _node_without_ros()
    target = {
        "label": "checkpoint_6",
        "metadata": {"order": 6},
        "pose": {
            "position": {"x": 2.0, "y": 3.0, "z": 0.0},
        },
    }

    scan_position, side, lane = node._scan_target(target)

    assert scan_position.scan_position_id == "bed_2_a_2"
    assert scan_position.bed_id == "2"
    assert scan_position.order == 6
    assert side == "a"
    assert lane == ""


def test_terminal_checkpoint_skips_perception():
    target = {
        "label": "checkpoint_13",
        "metadata": {
            "order": 13,
            "terminal": True,
            "run_perception": False,
        },
    }

    assert DemoCheckpointMissionNode._is_terminal_target(target)


def test_regular_checkpoint_is_not_terminal():
    target = {
        "label": "checkpoint_12",
        "metadata": {
            "order": 12,
            "terminal": False,
            "run_perception": True,
        },
    }

    assert not DemoCheckpointMissionNode._is_terminal_target(target)

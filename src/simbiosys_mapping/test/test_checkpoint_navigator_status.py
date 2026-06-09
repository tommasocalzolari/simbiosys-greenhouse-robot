import json

from geometry_msgs.msg import PoseStamped

from simbiosys_mapping.checkpoint_navigator_node import CheckpointNavigatorNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


def _pose():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    pose.pose.orientation.w = 1.0
    return pose


def test_checkpoint_status_includes_structured_targets():
    node = object.__new__(CheckpointNavigatorNode)
    node._route = [
        {
            "label": "checkpoint_1",
            "pose": _pose(),
            "metadata": {
                "bed_id": "1",
                "side": "a",
                "scan_position_id": "bed_1_a_1",
                "order": 1,
                "target_distance_m": 0.42,
                "terminal": False,
                "run_perception": True,
            },
        }
    ]
    node._next_index = 0
    node._active_target = None
    node._state = "waiting"
    node._status_pub = _Publisher()
    node.get_logger = lambda: _Logger()

    node._publish_status("ready", "ready")

    status = json.loads(node._status_pub.messages[-1].data)
    assert status["event"] == "ready"
    assert status["route_length"] == 1
    assert status["next_target"]["label"] == "checkpoint_1"
    assert status["next_target"]["metadata"]["bed_id"] == "1"
    assert status["next_target"]["metadata"]["side"] == "a"
    assert not status["next_target"]["metadata"]["terminal"]
    assert status["next_target"]["metadata"]["run_perception"]
    assert status["next_target"]["pose"]["position"]["x"] == 1.0


def test_checkpoint_metadata_preserves_terminal_flags():
    checkpoint = {
        "order": 13,
        "terminal": True,
        "run_perception": False,
    }

    metadata = CheckpointNavigatorNode._metadata_from_checkpoint(
        object.__new__(CheckpointNavigatorNode),
        checkpoint,
        "checkpoint_13",
        13,
    )

    assert metadata["terminal"]
    assert not metadata["run_perception"]

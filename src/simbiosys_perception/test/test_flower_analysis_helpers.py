from builtin_interfaces.msg import Time

from simbiosys_interfaces.msg import FlowerData, ScanPosition
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

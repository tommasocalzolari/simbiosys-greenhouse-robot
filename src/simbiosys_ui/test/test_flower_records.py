from builtin_interfaces.msg import Time

from simbiosys_interfaces.msg import PlantHealth
from simbiosys_ui.ui_node import UiNode


def _ui_without_ros():
    node = object.__new__(UiNode)
    node._plants = {}
    node._external_report = None
    return node


def _plant_health():
    msg = PlantHealth()
    msg.flower_id = "1:a:bed_1_a_1"
    msg.bed_id = "1"
    msg.color = "light_pink"
    msg.flower_detected = True
    msg.detected_colors = [
        "light_pink",
        "white",
        "light_pink",
    ]
    msg.detected_heights_cm = [8.6, 7.9, 4.4]
    msg.detected_confidences = [0.8, 0.7, 0.6]
    msg.last_scan_time = Time(sec=12, nanosec=0)
    msg.notes = "bed_side:a; lane:left; detected=3"
    return msg


def test_typed_plant_health_creates_one_record_per_detection():
    node = _ui_without_ros()

    node._on_typed_plant_health(_plant_health())

    assert list(node._plants) == [
        "1:a:bed_1_a_1:01",
        "1:a:bed_1_a_1:02",
        "1:a:bed_1_a_1:03",
    ]
    assert [plant["color"] for plant in node._plants.values()] == [
        "light_pink",
        "white",
        "light_pink",
    ]
    assert [plant["height_cm"] for plant in node._plants.values()] == [
        8.6,
        7.9,
        4.4,
    ]
    assert {plant["bed_side"] for plant in node._plants.values()} == {"a"}
    assert {plant["lane"] for plant in node._plants.values()} == {"left"}


def test_no_detection_removes_previous_scan_records():
    node = _ui_without_ros()
    node._on_typed_plant_health(_plant_health())
    no_detection = PlantHealth()
    no_detection.flower_id = "1:a:bed_1_a_1"
    no_detection.bed_id = "1"
    no_detection.flower_detected = False

    node._on_typed_plant_health(no_detection)

    assert node._plants == {}

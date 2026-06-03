import json
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from simbiosys_interfaces.msg import (
    BedObservation,
    FlowerData,
    PlantAnalysis,
    PlantHealth,
)


@dataclass
class TrackedFlower:
    color: str
    last_x: float
    last_y: float
    counted: bool = True
    frames_unseen: int = 0


class PlantAnalysisNode(Node):
    """Placeholder plant analysis node."""

    def __init__(self) -> None:
        super().__init__("plant_analysis_node")
        self.declare_parameter("tracking_threshold", 80)
        self.declare_parameter("max_frames_unseen", 10)

        self._publisher = self.create_publisher(
            PlantAnalysis,
            "simbiosys/plant_analysis",
            10,
        )
        self._plant_health_publisher = self.create_publisher(
            PlantHealth,
            "simbiosys/plant_health",
            10,
        )
        self._flower_counts_publisher = self.create_publisher(
            String,
            "/simbiosys/flower_counts",
            10,
        )
        self.create_subscription(
            FlowerData,
            "simbiosys/flower_data",
            self._on_flower_data,
            10,
        )
        self.create_subscription(
            Bool,
            "/simbiosys/reset_counter",
            self._on_reset_counter,
            10,
        )
        self.create_subscription(
            Bool,
            "/simbiosys/bug_detected",
            self._on_bug_detected,
            10,
        )
        self.create_subscription(
            Int32,
            "/simbiosys/current_bed_id",
            self._on_current_bed_id,
            10,
        )
        self.create_subscription(
            BedObservation,
            "simbiosys/bed_observation",
            self._on_bed_observation,
            10,
        )
        self._latest_flower_data: FlowerData | None = None
        self._current_bed_id = -1
        self._bug_detected = False
        self._bed_flower_counts: dict[int, dict[str, int]] = {}
        self._bed_tracked_flowers: dict[int, list[TrackedFlower]] = {}
        self._tracked_flowers: list[TrackedFlower] = []
        self._flower_counts = self._empty_flower_counts()
        self._timer = self.create_timer(5.0, self._on_timer)
        self.declare_parameter("default_bed_id", "A")
        self.declare_parameter("default_flower_id", "A1")
        self.declare_parameter("harvest_height_cm", 35.0)
        self._flower_counts_timer = self.create_timer(
            1.0,
            self._publish_flower_counts,
        )

        # TODO: Add maturity estimation.
        self.get_logger().info("Plant analysis placeholder started")

    def _on_flower_data(self, msg: FlowerData) -> None:
        self._latest_flower_data = msg
        if msg.detected:
            self._track_flower(msg)
        else:
            self._age_tracked_flowers()

    def _track_flower(self, msg: FlowerData) -> None:
        tracking_threshold = self.get_parameter(
            "tracking_threshold"
        ).get_parameter_value().integer_value
        flower_x = msg.position.x
        flower_y = msg.position.y
        flower_color = msg.label

        matched_flower = None
        for tracked_flower in self._tracked_flowers:
            distance_px = (
                (flower_x - tracked_flower.last_x) ** 2
                + (flower_y - tracked_flower.last_y) ** 2
            ) ** 0.5
            if distance_px < tracking_threshold:
                matched_flower = tracked_flower
                break

        if matched_flower is not None:
            match_status = (
                f"matched existing {matched_flower.color} "
                f"at center=({matched_flower.last_x:.1f},"
                f"{matched_flower.last_y:.1f})"
            )
            matched_flower.last_x = flower_x
            matched_flower.last_y = flower_y
            matched_flower.frames_unseen = 0
        else:
            match_status = "created new tracked flower"
            self._tracked_flowers.append(TrackedFlower(flower_color, flower_x, flower_y))
            if flower_color in self._flower_counts:
                self._flower_counts[flower_color] += 1
            else:
                self.get_logger().warning(
                    f"Detected flower with untracked color '{flower_color}'"
                )

        self._prune_unseen_flowers()
        self.get_logger().info(
            f"FlowerData color={flower_color} x={flower_x:.1f}: {match_status}"
        )
        self._log_tracked_flowers()

    def _log_tracked_flowers(self) -> None:
        if not self._tracked_flowers:
            self.get_logger().info("Tracked flowers: []")
            return

        tracked_state = [
            (
                f"{index}:color={tracked_flower.color},"
                f"center=({tracked_flower.last_x:.1f},"
                f"{tracked_flower.last_y:.1f}),"
                f"unseen={tracked_flower.frames_unseen}"
            )
            for index, tracked_flower in enumerate(self._tracked_flowers)
        ]
        self.get_logger().info(f"Tracked flowers: [{'; '.join(tracked_state)}]")

    def _age_tracked_flowers(self) -> None:
        for tracked_flower in self._tracked_flowers:
            tracked_flower.frames_unseen += 1
        self._prune_unseen_flowers()

    def _prune_unseen_flowers(self) -> None:
        max_frames_unseen = self.get_parameter(
            "max_frames_unseen"
        ).get_parameter_value().integer_value
        self._tracked_flowers = [
            tracked_flower
            for tracked_flower in self._tracked_flowers
            if tracked_flower.frames_unseen <= max_frames_unseen
        ]

    def _on_reset_counter(self, msg: Bool) -> None:
        if msg.data:
            self._tracked_flowers = []
            self._flower_counts = self._empty_flower_counts()
            self._save_current_bed_state()
            self.get_logger().info(
                f"Reset flower counters for bed {self._current_bed_id}"
            )

    def _on_bug_detected(self, msg: Bool) -> None:
        self._bug_detected = bool(msg.data)

    def _on_current_bed_id(self, msg: Int32) -> None:
        self._set_current_bed_id(msg.data)

    def _on_bed_observation(self, msg: BedObservation) -> None:
        self._set_current_bed_id(msg.bed_id)

    def _set_current_bed_id(self, bed_id: int) -> None:
        if bed_id == self._current_bed_id:
            return

        old_bed_id = self._current_bed_id
        self._save_current_bed_state()
        self._current_bed_id = bed_id
        self._load_current_bed_state()
        self.get_logger().info(
            f"Switched flower counters from bed {old_bed_id} to bed {bed_id}"
        )

    def _save_current_bed_state(self) -> None:
        self._bed_flower_counts[self._current_bed_id] = dict(self._flower_counts)
        self._bed_tracked_flowers[self._current_bed_id] = list(self._tracked_flowers)

    def _load_current_bed_state(self) -> None:
        self._flower_counts = self._empty_flower_counts()
        self._flower_counts.update(
            self._bed_flower_counts.get(self._current_bed_id, {})
        )
        self._tracked_flowers = list(
            self._bed_tracked_flowers.get(self._current_bed_id, [])
        )

    def _empty_flower_counts(self) -> dict[str, int]:
        return {
            "magenta": 0,
            "light": 0,
            "white": 0,
            "light_pink": 0,
        }

    def _on_timer(self) -> None:
        analysis = PlantAnalysis()
        analysis.bugs_detected = self._bug_detected
        analysis.fully_grown = False

        if self._latest_flower_data is None:
            analysis.plant_detected = False
            analysis.height = 0.0
            analysis.color = "unknown"
            analysis.position = Point()
            analysis.message = "Placeholder plant analysis"
        else:
            flower_data = self._latest_flower_data
            analysis.plant_detected = flower_data.detected
            analysis.height = flower_data.position.z
            analysis.color = flower_data.label
            analysis.position = flower_data.position
            analysis.message = flower_data.message

        self._publisher.publish(analysis)
        self._plant_health_publisher.publish(self._plant_health_from_analysis(analysis))
        self.get_logger().info("Published plant analysis")

    def _plant_health_from_analysis(self, analysis: PlantAnalysis) -> PlantHealth:
        msg = PlantHealth()
        msg.bed_id = self._bed_id_for_health()
        msg.flower_id = self._flower_id_for_health(msg.bed_id)
        msg.height_cm = float(analysis.height)
        msg.color = analysis.color or "unknown"
        msg.bug_detected = bool(analysis.bugs_detected)
        msg.flower_detected = bool(analysis.plant_detected)
        harvest_height_cm = (
            self.get_parameter("harvest_height_cm").get_parameter_value().double_value
        )
        msg.ready_for_harvest = bool(
            analysis.fully_grown or analysis.height >= harvest_height_cm
        )
        msg.health = self._health_label(analysis)
        msg.growth_stage = "mature" if msg.ready_for_harvest else "growing"
        msg.confidence = 1.0 if analysis.plant_detected else 0.0
        msg.last_scan_time = self.get_clock().now().to_msg()
        msg.notes = analysis.message
        msg.position = analysis.position
        return msg

    def _health_label(self, analysis: PlantAnalysis) -> str:
        if not analysis.plant_detected:
            return "unknown"
        if analysis.bugs_detected:
            return "critical"
        return "healthy"

    def _bed_id_for_health(self) -> str:
        if self._current_bed_id >= 0:
            return str(self._current_bed_id)
        return self.get_parameter("default_bed_id").get_parameter_value().string_value

    def _flower_id_for_health(self, bed_id: str) -> str:
        default_flower_id = (
            self.get_parameter("default_flower_id").get_parameter_value().string_value
        )
        if self._latest_flower_data is None or not self._latest_flower_data.detected:
            return default_flower_id
        return f"{bed_id}-latest"

    def _publish_flower_counts(self) -> None:
        flower_counts = {
            "bed_id": self._current_bed_id,
            "magenta": self._flower_counts.get("magenta", 0),
            "light": self._flower_counts.get("light", 0),
            "white": self._flower_counts.get("white", 0),
            "light_pink": self._flower_counts.get("light_pink", 0),
        }
        flower_counts["total"] = (
            flower_counts["magenta"]
            + flower_counts["light"]
            + flower_counts["white"]
            + flower_counts["light_pink"]
        )

        msg = String()
        msg.data = json.dumps(flower_counts)
        self._flower_counts_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlantAnalysisNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

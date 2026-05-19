import json
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from simbiosys_interfaces.msg import FlowerData, PlantAnalysis


@dataclass
class TrackedFlower:
    color: str
    last_x: float
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
            Int32,
            "/simbiosys/current_bed_id",
            self._on_current_bed_id,
            10,
        )
        self._latest_flower_data: FlowerData | None = None
        self._current_bed_id = -1
        self._tracked_flowers: list[TrackedFlower] = []
        self._flower_counts = {
            "magenta": 0,
            "light_pink": 0,
            "white": 0,
        }
        self._timer = self.create_timer(5.0, self._on_timer)
        self._flower_counts_timer = self.create_timer(
            1.0,
            self._publish_flower_counts,
        )

        # TODO: Add bug detection and maturity estimation.
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

        matched_flower = None
        for tracked_flower in self._tracked_flowers:
            if abs(flower_x - tracked_flower.last_x) < tracking_threshold:
                matched_flower = tracked_flower
                break

        for tracked_flower in self._tracked_flowers:
            if tracked_flower is matched_flower:
                tracked_flower.last_x = flower_x
                tracked_flower.frames_unseen = 0
            else:
                tracked_flower.frames_unseen += 1

        if matched_flower is None:
            flower_color = msg.label
            self._tracked_flowers.append(TrackedFlower(flower_color, flower_x))
            if flower_color in self._flower_counts:
                self._flower_counts[flower_color] += 1
            else:
                self.get_logger().warning(
                    f"Detected flower with untracked color '{flower_color}'"
                )

        self._prune_unseen_flowers()

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
            self._tracked_flowers.clear()
            for color in self._flower_counts:
                self._flower_counts[color] = 0
            self.get_logger().info("Reset flower counters")

    def _on_current_bed_id(self, msg: Int32) -> None:
        self._current_bed_id = msg.data

    def _on_timer(self) -> None:
        analysis = PlantAnalysis()
        analysis.bugs_detected = False
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
        self.get_logger().info("Published plant analysis")

    def _publish_flower_counts(self) -> None:
        flower_counts = {
            "bed_id": self._current_bed_id,
            "magenta": self._flower_counts["magenta"],
            "light_pink": self._flower_counts["light_pink"],
            "white": self._flower_counts["white"],
        }
        flower_counts["total"] = (
            flower_counts["magenta"]
            + flower_counts["light_pink"]
            + flower_counts["white"]
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

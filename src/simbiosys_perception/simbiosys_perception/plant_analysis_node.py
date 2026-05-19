import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from simbiosys_interfaces.msg import FlowerData, PlantAnalysis, PlantHealth


class PlantAnalysisNode(Node):
    """Placeholder plant analysis node."""

    def __init__(self) -> None:
        super().__init__("plant_analysis_node")
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
        self.create_subscription(
            FlowerData,
            "simbiosys/flower_data",
            self._on_flower_data,
            10,
        )
        self._latest_flower_data: FlowerData | None = None
        self._timer = self.create_timer(5.0, self._on_timer)
        self.declare_parameter("default_bed_id", "A")
        self.declare_parameter("default_flower_id", "A1")
        self.declare_parameter("harvest_height_cm", 35.0)

        # TODO: Add bug detection and maturity estimation.
        self.get_logger().info("Plant analysis placeholder started")

    def _on_flower_data(self, msg: FlowerData) -> None:
        self._latest_flower_data = msg

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
        self._plant_health_publisher.publish(self._plant_health_from_analysis(analysis))
        self.get_logger().info("Published plant analysis")

    def _plant_health_from_analysis(self, analysis: PlantAnalysis) -> PlantHealth:
        msg = PlantHealth()
        msg.bed_id = (
            self.get_parameter("default_bed_id").get_parameter_value().string_value
        )
        msg.flower_id = (
            self.get_parameter("default_flower_id").get_parameter_value().string_value
        )
        msg.height_cm = float(analysis.height)
        msg.color = analysis.color or "unknown"
        msg.bug_detected = bool(analysis.bugs_detected)
        msg.flower_detected = bool(analysis.plant_detected)
        msg.ready_for_harvest = bool(
            analysis.fully_grown
            or analysis.height
            >= self.get_parameter("harvest_height_cm").get_parameter_value().double_value
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

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from simbiosys_interfaces.msg import FlowerData, PlantAnalysis


class PlantAnalysisNode(Node):
    """Placeholder plant analysis node."""

    def __init__(self) -> None:
        super().__init__("plant_analysis_node")
        self._publisher = self.create_publisher(
            PlantAnalysis,
            "simbiosys/plant_analysis",
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
        self.get_logger().info("Published plant analysis")


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

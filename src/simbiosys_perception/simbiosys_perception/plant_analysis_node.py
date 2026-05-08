import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from simbiosys_interfaces.msg import PlantAnalysis


class PlantAnalysisNode(Node):
    """Placeholder plant analysis node."""

    def __init__(self) -> None:
        super().__init__("plant_analysis_node")
        self._publisher = self.create_publisher(
            PlantAnalysis,
            "simbiosys/plant_analysis",
            10,
        )
        self._timer = self.create_timer(5.0, self._on_timer)

        # TODO: Subscribe to camera topics and run plant detection.
        # TODO: Add real height, color, maturity, and pose estimation.
        self.get_logger().info("Plant analysis placeholder started")

    def _on_timer(self) -> None:
        analysis = PlantAnalysis()
        analysis.plant_detected = False
        analysis.bugs_detected = False
        analysis.fully_grown = False
        analysis.height = 0.0
        analysis.color = "unknown"
        analysis.position = Point()
        analysis.message = "Placeholder plant analysis"

        self._publisher.publish(analysis)
        self.get_logger().info("Published placeholder plant analysis")


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

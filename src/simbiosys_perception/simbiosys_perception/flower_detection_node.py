import rclpy
from rclpy.node import Node

from simbiosys_interfaces.msg import FlowerData


class FlowerDetectionNode(Node):
    """Placeholder flower detector that keeps the ROS interface stable."""

    def __init__(self) -> None:
        super().__init__("flower_detection_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "simbiosys/flower_data")
        self.declare_parameter("publish_period_sec", 5.0)

        self._image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        period = (
            self.get_parameter("publish_period_sec").get_parameter_value().double_value
        )

        self._publisher = self.create_publisher(FlowerData, output_topic, 10)
        self.create_timer(period, self._on_timer)

        # TODO: Add image subscription with cv_bridge/OpenCV once the camera
        # topic and lightweight detection approach are confirmed.
        self.get_logger().info(
            f"Flower detection placeholder publishing {output_topic}; "
            f"future input topic is {self._image_topic}"
        )

    def _on_timer(self) -> None:
        msg = FlowerData()
        msg.detected = False
        msg.confidence = 0.0
        msg.label = "placeholder"
        msg.message = (
            "Flower detection placeholder; cv_bridge/OpenCV integration comes later"
        )
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlowerDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

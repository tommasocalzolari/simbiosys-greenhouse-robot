#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class SaveFrameWristNode(Node):
    def __init__(self) -> None:
        super().__init__("save_frame_wrist")
        self._saved = False
        self._subscription = self.create_subscription(
            CompressedImage,
            "/gripper_camera/image_raw/compressed",
            self._on_image,
            10,
        )

    def _on_image(self, msg: CompressedImage) -> None:
        if self._saved:
            return

        image_buffer = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(image_buffer, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Could not decode compressed image")
            return

        output_path = (
            Path.home() / "Downloads" / f"frame_wrist_{datetime.now():%H%M%S}.jpg"
        )
        if not cv2.imwrite(str(output_path), frame):
            self.get_logger().error(f"Could not save {output_path}")
            rclpy.shutdown()
            return

        self._saved = True
        print(f"Saved to {output_path}", flush=True)
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = SaveFrameWristNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

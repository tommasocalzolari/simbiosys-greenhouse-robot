import math
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class GetMapNode(Node):
    """Monitor SLAM topics and save the latest occupancy grid map."""

    def __init__(self) -> None:
        super().__init__("getmap_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("output_dir", "maps")
        self.declare_parameter("map_name", "mirte_map")
        self.declare_parameter("occupied_thresh", 0.65)
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("status_period", 5.0)
        self.declare_parameter("auto_save_period", 20.0)
        self.declare_parameter("save_on_shutdown", False)

        self._seen_scan = False
        self._seen_odom = False
        self._map: OccupancyGrid | None = None

        self.create_subscription(
            LaserScan,
            self._string_parameter("scan_topic"),
            self._on_scan,
            10,
        )
        self.create_subscription(
            Odometry,
            self._string_parameter("odom_topic"),
            self._on_odom,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self._string_parameter("map_topic"),
            self._on_map,
            10,
        )

        self.create_service(Trigger, "~/save_map", self._on_save_map)
        self.create_timer(
            self._double_parameter("status_period"),
            self._log_status,
        )
        auto_save_period = self._double_parameter("auto_save_period")
        if auto_save_period > 0.0:
            self.create_timer(auto_save_period, self._auto_save_map)

        self.get_logger().info(
            "GetMap node started. Save the current map with: "
            "ros2 service call /getmap_node/save_map std_srvs/srv/Trigger {}"
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _on_scan(self, _msg: LaserScan) -> None:
        self._seen_scan = True

    def _on_odom(self, _msg: Odometry) -> None:
        self._seen_odom = True

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_save_map(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._map is None:
            response.success = False
            response.message = "No /map message received yet."
            return response

        yaml_path = self._save_map(self._map)
        response.success = True
        response.message = f"Saved map to {yaml_path}"
        return response

    def _log_status(self) -> None:
        map_ready = self._map is not None
        self.get_logger().info(
            "Mapping inputs: "
            f"scan={self._seen_scan}, odom={self._seen_odom}, map={map_ready}"
        )

    def _auto_save_map(self) -> None:
        if self._map is None:
            return
        self._save_map(self._map)

    def _save_map(self, msg: OccupancyGrid) -> Path:
        output_dir = Path(self._string_parameter("output_dir")).expanduser()
        map_name = self._string_parameter("map_name")
        occupied_thresh = self._double_parameter("occupied_thresh")
        free_thresh = self._double_parameter("free_thresh")

        output_dir.mkdir(parents=True, exist_ok=True)
        pgm_path = output_dir / f"{map_name}.pgm"
        yaml_path = output_dir / f"{map_name}.yaml"

        self._write_pgm(msg, pgm_path, occupied_thresh, free_thresh)
        self._write_yaml(msg, yaml_path, pgm_path.name, occupied_thresh, free_thresh)
        self.get_logger().info(f"Saved map: {yaml_path}")
        return yaml_path

    def _write_pgm(
        self,
        msg: OccupancyGrid,
        path: Path,
        occupied_thresh: float,
        free_thresh: float,
    ) -> None:
        width = msg.info.width
        height = msg.info.height
        occupied_limit = int(occupied_thresh * 100)
        free_limit = int(free_thresh * 100)

        with path.open("wb") as pgm_file:
            pgm_file.write(
                f"P5\n# CREATOR: simbiosys_mapping getmap_node\n"
                f"{width} {height}\n255\n".encode("ascii")
            )
            for image_y in range(height):
                map_y = height - image_y - 1
                for x in range(width):
                    value = msg.data[x + (map_y * width)]
                    if value < 0:
                        pixel = 205
                    elif value >= occupied_limit:
                        pixel = 0
                    elif value <= free_limit:
                        pixel = 254
                    else:
                        pixel = 205
                    pgm_file.write(bytes([pixel]))

    def _write_yaml(
        self,
        msg: OccupancyGrid,
        path: Path,
        image_name: str,
        occupied_thresh: float,
        free_thresh: float,
    ) -> None:
        origin = msg.info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        path.write_text(
            "\n".join(
                [
                    f"image: {image_name}",
                    "mode: trinary",
                    f"resolution: {msg.info.resolution}",
                    (
                        "origin: "
                        f"[{origin.position.x}, {origin.position.y}, {yaw}]"
                    ),
                    "negate: 0",
                    f"occupied_thresh: {occupied_thresh}",
                    f"free_thresh: {free_thresh}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

    def _yaw_from_quaternion(self, orientation) -> float:
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def save_on_shutdown(self) -> None:
        if self._bool_parameter("save_on_shutdown") and self._map is not None:
            self._save_map(self._map)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GetMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.save_on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

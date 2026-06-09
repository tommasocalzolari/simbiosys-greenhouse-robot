#!/usr/bin/env python3
"""Test-only ROS 2 data publisher for the SimBioSys UI.

This standalone tool publishes artificial UI test data over ROS topics. It is
not part of the UI package and must not be used as UI fallback logic.
"""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from typing import Any

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from simbiosys_interfaces.action import ExecuteBehavior
from simbiosys_interfaces.msg import (
    BedObservation,
    BehaviorType,
    DetectedTag,
    HarvestStatus,
    MappingStatus,
    PlantAnalysis,
    PlantHealth,
    ScanProgress,
    TaskStatus,
)
from simbiosys_interfaces.srv import SendNamedArmPose, SetRobotMode
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


MAP_WIDTH = 120
MAP_HEIGHT = 80
MAP_RESOLUTION = 0.1
MAP_ORIGIN_X = -1.0
MAP_ORIGIN_Y = -1.0


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def world_to_grid(x: float, y: float) -> tuple[int, int]:
    gx = int(round((x - MAP_ORIGIN_X) / MAP_RESOLUTION))
    gy = int(round((y - MAP_ORIGIN_Y) / MAP_RESOLUTION))
    return gx, gy


def set_cell(data: list[int], x: int, y: int, value: int) -> None:
    if 0 <= x < MAP_WIDTH and 0 <= y < MAP_HEIGHT:
        data[y * MAP_WIDTH + x] = value


def fill_rect(data: list[int], x0: int, y0: int, x1: int, y1: int, value: int) -> None:
    left, right = sorted((max(0, x0), min(MAP_WIDTH - 1, x1)))
    bottom, top = sorted((max(0, y0), min(MAP_HEIGHT - 1, y1)))
    for y in range(bottom, top + 1):
        for x in range(left, right + 1):
            set_cell(data, x, y, value)


def fill_world_rect(
    data: list[int], x0: float, y0: float, x1: float, y1: float, value: int
) -> None:
    gx0, gy0 = world_to_grid(x0, y0)
    gx1, gy1 = world_to_grid(x1, y1)
    fill_rect(data, gx0, gy0, gx1, gy1, value)


def make_greenhouse_map() -> list[int]:
    data = [0] * (MAP_WIDTH * MAP_HEIGHT)

    fill_rect(data, 0, 0, MAP_WIDTH - 1, 2, 100)
    fill_rect(data, 0, MAP_HEIGHT - 3, MAP_WIDTH - 1, MAP_HEIGHT - 1, 100)
    fill_rect(data, 0, 0, 2, MAP_HEIGHT - 1, 100)
    fill_rect(data, MAP_WIDTH - 3, 0, MAP_WIDTH - 1, MAP_HEIGHT - 1, 100)

    beds = [
        (1.0, 1.2, 2.4, 6.0),
        (4.1, 1.2, 5.5, 6.0),
        (7.2, 1.2, 8.6, 6.0),
    ]
    for bed in beds:
        fill_world_rect(data, *bed, 70)

    obstacles = [
        (3.0, 2.0, 3.5, 2.6),
        (6.2, 4.7, 6.9, 5.3),
        (9.4, 1.4, 9.9, 2.0),
    ]
    for obstacle in obstacles:
        fill_world_rect(data, *obstacle, 100)

    noise_blobs = [
        (2.9, 6.5),
        (5.9, 0.8),
        (9.8, 5.8),
        (0.7, 3.4),
    ]
    for x, y in noise_blobs:
        gx, gy = world_to_grid(x, y)
        fill_rect(data, gx - 1, gy - 1, gx + 1, gy + 1, 100)

    return data


def make_mock_plant(
    bed_id: int,
    bed_side: str,
    lane: str,
    plant_index: int,
    color: str,
    confidence: float = 0.9,
) -> dict[str, Any]:
    x_base = 1.0 + (bed_id - 1) * 2.5
    x_offset = 0.32 if bed_side == "b" else 0.0
    y_base = 2.0 if lane == "left" else 4.8
    height_cm = 15.0 + bed_id * 2.5 + plant_index * 1.1 + (2.0 if bed_side == "b" else 0.0)
    return {
        "flower_id": f"{bed_id}:{bed_side}:{lane}:{plant_index:02d}",
        "bed_side": bed_side,
        "lane": lane,
        "side": lane,
        "height_cm": round(height_cm, 1),
        "color": color,
        "health": "",
        "growth_stage": "",
        "bug_detected": False,
        "flower_detected": True,
        "ready_for_harvest": False,
        "confidence": confidence,
        "notes": f"bed_side:{bed_side} lane:{lane}",
        "position": (x_base + x_offset + plant_index * 0.08, y_base + plant_index * 0.12, height_cm),
    }


def count_colors(plants: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"magenta": 0, "light_pink": 0, "white": 0, "yellow": 0, "red": 0}
    for plant in plants:
        color = plant.get("color")
        if color in counts:
            counts[color] += 1
    return counts


def make_mock_perception_beds() -> list[dict[str, Any]]:
    """Realistic perception and behavior-shaped data for UI validation."""
    bed_specs = [
        {
            "bed_id": 1,
            "co2_ppm": 620,
            "humidity_percent": 62,
            "bugs_detected": False,
            "compartments": [
                {"bed_side": "a", "lane": "left", "color": "magenta", "count": 4},
                {"bed_side": "a", "lane": "right", "color": "magenta", "count": 3},
                {"bed_side": "b", "lane": "left", "color": "white", "count": 2},
                {"bed_side": "b", "lane": "right", "color": "white", "count": 4},
            ],
        },
        {
            "bed_id": 2,
            "co2_ppm": 585,
            "humidity_percent": 82,
            "bugs_detected": True,
            "compartments": [
                {"bed_side": "a", "lane": "left", "color": "light_pink", "count": 3},
                {"bed_side": "a", "lane": "right", "color": "light_pink", "count": 4},
                {"bed_side": "b", "lane": "left", "color": "magenta", "count": 2},
                {"bed_side": "b", "lane": "right", "color": "magenta", "count": 2},
            ],
        },
        {
            "bed_id": 3,
            "co2_ppm": 1320,
            "humidity_percent": 58,
            "bugs_detected": False,
            "compartments": [
                {"bed_side": "a", "lane": "left", "color": "white", "count": 1},
                {"bed_side": "a", "lane": "right", "color": "white", "count": 0},
                {"bed_side": "b", "lane": "left", "color": "light_pink", "count": 1},
                {"bed_side": "b", "lane": "right", "color": "light_pink", "count": 0},
            ],
        },
        {
            "bed_id": 4,
            "co2_ppm": 2150,
            "humidity_percent": 28,
            "bugs_detected": False,
            "compartments": [
                {"bed_side": "a", "lane": "left", "color": "yellow", "count": 6},
                {"bed_side": "a", "lane": "right", "color": "yellow", "count": 5},
                {"bed_side": "b", "lane": "left", "color": "red", "count": 5},
                {"bed_side": "b", "lane": "right", "color": "red", "count": 6},
            ],
        },
    ]

    beds = []
    for spec in bed_specs:
        plants = []
        compartments = []
        for compartment in spec["compartments"]:
            compartment_plants = []
            for index in range(1, compartment["count"] + 1):
                plants.append(
                    make_mock_plant(
                        bed_id=spec["bed_id"],
                        bed_side=compartment["bed_side"],
                        lane=compartment["lane"],
                        plant_index=index,
                        color=compartment["color"],
                        confidence=max(0.55, 0.96 - index * 0.025),
                    )
                )
                compartment_plants.append(plants[-1])
            compartments.append(
                {
                    **compartment,
                    "heights_cm": [plant["height_cm"] for plant in compartment_plants],
                }
            )
        beds.append(
            {
                "bed_id": spec["bed_id"],
                "co2_ppm": spec["co2_ppm"],
                "humidity_percent": spec["humidity_percent"],
                "bugs_detected": spec["bugs_detected"],
                "flower_counts": count_colors(plants),
                "compartments": compartments,
                "plants": plants,
            }
        )
    return beds


class UiTestDataPublisher(Node):
    """Publishes artificial ROS 2 test data for UI integration testing."""

    def __init__(
        self,
        map_period: float,
        bed_period: float,
        odom_period: float,
        once: bool,
    ) -> None:
        super().__init__("simbiosys_ui_test_data_publisher")
        self._start_time = self.get_clock().now()
        self._once = once
        self._greenhouse_map = make_greenhouse_map()
        self._perception_beds = make_mock_perception_beds()
        self._behavior_bed_index = 0

        self._map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self._odom_pub = self.create_publisher(
            Odometry, "/mirte_base_controller/odom", 10
        )
        self._amcl_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10
        )
        self._mapping_status_pub = self.create_publisher(
            MappingStatus, "simbiosys/mapping_status", 10
        )
        self._task_status_pub = self.create_publisher(
            TaskStatus, "simbiosys/task_status", 10
        )
        self._scan_progress_pub = self.create_publisher(
            ScanProgress, "simbiosys/scan_progress", 10
        )
        self._harvest_status_pub = self.create_publisher(
            HarvestStatus, "simbiosys/harvest_status", 10
        )
        self._battery_pub = self.create_publisher(
            BatteryState, "/io/power/power_watcher", 10
        )
        self._bed_environment_pub = self.create_publisher(String, "/bed_environment", 10)
        self._bed_observation_pub = self.create_publisher(
            BedObservation, "simbiosys/bed_observation", 10
        )
        self._current_bed_pub = self.create_publisher(
            Int32, "/simbiosys/current_bed_id", 10
        )
        self._plant_analysis_pub = self.create_publisher(
            PlantAnalysis, "simbiosys/plant_analysis", 10
        )
        self._plant_health_pub = self.create_publisher(
            PlantHealth, "simbiosys/plant_health", 10
        )
        self._plant_health_json_pub = self.create_publisher(String, "/plant_health", 10)
        self._plant_health_report_pub = self.create_publisher(
            String, "/plant_health_report", 10
        )
        self._flower_counts_pub = self.create_publisher(
            String, "/simbiosys/flower_counts", 10
        )
        self._behavior_action_server = ActionServer(
            self,
            ExecuteBehavior,
            "simbiosys/execute_behavior",
            self._on_execute_behavior,
        )
        self.create_service(SetRobotMode, "simbiosys/set_robot_mode", self._on_set_robot_mode)
        self.create_service(
            SendNamedArmPose,
            "simbiosys/send_named_arm_pose",
            self._on_send_named_arm_pose,
        )
        self.create_service(Trigger, "/mapping/start", self._on_start_mapping)
        self.create_service(Trigger, "/mapping/done", self._on_done_mapping)
        self.create_service(
            Trigger, "/mapping/save_safe_map", self._on_save_safe_map
        )

        if once:
            self.publish_all()
            self.get_logger().info("Published one UI test data set.")
        else:
            self.create_timer(map_period, self.publish_map)
            self.create_timer(bed_period, self.publish_perception_data)
            self.create_timer(bed_period, self.publish_behavior_status)
            self.create_timer(20.0, self.publish_mapping_status)
            self.create_timer(60.0, self.publish_battery)
            self.create_timer(odom_period, self.publish_odometry)
            self.publish_all()
            self.get_logger().info(
                "Publishing test-only UI data on /map, /mirte_base_controller/odom, "
                "/amcl_pose, "
                "simbiosys/mapping_status, simbiosys/scan_progress, simbiosys/harvest_status, "
                "/bed_environment, /io/power/power_watcher, /simbiosys/current_bed_id, "
                "simbiosys/bed_observation, simbiosys/plant_analysis, "
                "simbiosys/plant_health, /plant_health_report, /plant_health, "
                "and /simbiosys/flower_counts. Providing test-only action "
                "simbiosys/execute_behavior, services simbiosys/set_robot_mode, "
                "simbiosys/send_named_arm_pose, /mapping/start, /mapping/done, "
                "and /mapping/save_safe_map."
            )

    def publish_all(self) -> None:
        self.publish_map()
        self.publish_perception_data()
        self.publish_behavior_status()
        self.publish_mapping_status()
        self.publish_battery()
        self.publish_odometry()

    def publish_map(self) -> None:
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info.map_load_time = message.header.stamp
        message.info.resolution = MAP_RESOLUTION
        message.info.width = MAP_WIDTH
        message.info.height = MAP_HEIGHT
        message.info.origin.position.x = MAP_ORIGIN_X
        message.info.origin.position.y = MAP_ORIGIN_Y
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.w = 1.0
        message.data = self._greenhouse_map
        self._map_pub.publish(message)

    def publish_odometry(self) -> None:
        elapsed = (
            self.get_clock().now().nanoseconds - self._start_time.nanoseconds
        ) / 1_000_000_000.0
        center_x = 0.2
        center_y = 0.8
        radius_x = 0.8
        radius_y = 1.0
        angular_speed = 0.08
        angle = elapsed * angular_speed
        x = center_x + radius_x * math.cos(angle)
        y = center_y + radius_y * math.sin(angle)
        yaw = angle + math.pi / 2.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = radius_x * angular_speed * -math.sin(angle)
        message.twist.twist.linear.y = radius_y * angular_speed * math.cos(angle)
        message.twist.twist.angular.z = angular_speed
        self._odom_pub.publish(message)

        amcl_message = PoseWithCovarianceStamped()
        amcl_message.header.stamp = message.header.stamp
        amcl_message.header.frame_id = "map"
        amcl_message.pose.pose.position.x = x
        amcl_message.pose.pose.position.y = y
        amcl_message.pose.pose.position.z = 0.0
        amcl_message.pose.pose.orientation.x = qx
        amcl_message.pose.pose.orientation.y = qy
        amcl_message.pose.pose.orientation.z = qz
        amcl_message.pose.pose.orientation.w = qw
        amcl_message.pose.covariance[0] = 0.05
        amcl_message.pose.covariance[7] = 0.05
        amcl_message.pose.covariance[35] = 0.02
        self._amcl_pose_pub.publish(amcl_message)

    def publish_mapping_status(self) -> None:
        message = MappingStatus()
        message.scan_seen = True
        message.odom_seen = True
        message.map_seen = True
        message.localized = True
        message.active_map = "test_greenhouse_map"
        message.message = "test mapping status: map, odom, and localization available"
        self._mapping_status_pub.publish(message)

    def publish_battery(self) -> None:
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = 0.76
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self._battery_pub.publish(message)

    def publish_behavior_status(self) -> None:
        if not self._perception_beds:
            return
        bed = self._perception_beds[self._behavior_bed_index % len(self._perception_beds)]
        self._behavior_bed_index += 1
        bed_id = str(bed["bed_id"])
        plants = list(bed["plants"])
        active_plant = plants[0] if plants else None

        task = TaskStatus()
        task.current_state = "SCANNING"
        task.active = True
        task.error = False
        task.message = f"test behavior scanning bed {bed_id}"
        self._task_status_pub.publish(task)

        scan = ScanProgress()
        scan.active_bed_id = bed_id
        scan.active_scan_position_id = f"{bed_id}:a"
        scan.active_flower_id = active_plant["flower_id"] if active_plant else ""
        scan.scan_index = len(plants)
        scan.scan_total = max(1, len(plants))
        scan.detection_status = "test_scan_ready"
        scan.retry_count = 0
        scan.error = False
        scan.message = f"test scan progress for bed {bed_id}"
        if active_plant:
            scan.latest_plant_health = self._make_plant_health(
                bed_id=int(bed["bed_id"]),
                plant=active_plant,
                scan_time=self.get_clock().now().to_msg(),
            )
        self._scan_progress_pub.publish(scan)

        harvest = HarvestStatus()
        harvest.active_bed_id = bed_id
        harvest.active_flower_id = active_plant["flower_id"] if active_plant else ""
        harvest.phase = "test_ready"
        harvest.alignment_status = "aligned"
        harvest.harvest_enabled = True
        harvest.success = True
        harvest.error = False
        harvest.message = f"test harvest status for bed {bed_id}"
        self._harvest_status_pub.publish(harvest)

    def publish_perception_data(self) -> None:
        ros_scan_time = self.get_clock().now().to_msg()
        report_total_flowers = 0

        for bed in self._perception_beds:
            bed_id = int(bed["bed_id"])
            flower_counts = dict(bed["flower_counts"])
            flower_total = int(sum(flower_counts.values()))
            report_total_flowers += flower_total

            current_bed = Int32()
            current_bed.data = bed_id
            self._current_bed_pub.publish(current_bed)

            observation = BedObservation()
            observation.bed_id = bed_id
            observation.visible = True
            observation.message = f"Visible bed tag {bed_id}"
            tag = DetectedTag()
            tag.id = bed_id
            tag.center_px = Point(x=320.0 + bed_id * 8.0, y=240.0, z=0.0)
            tag.area = 1200.0
            tag.confidence = 1.0
            observation.tags = [tag]
            self._bed_observation_pub.publish(observation)

            counts_message = String()
            counts_message.data = json.dumps(
                {
                    "bed_id": bed_id,
                    **flower_counts,
                    "total": flower_total,
                }
            )
            self._flower_counts_pub.publish(counts_message)

            environment_message = String()
            environment_message.data = json.dumps(
                {
                    "bed_id": bed_id,
                    "co2_ppm": bed["co2_ppm"],
                    "humidity_percent": bed["humidity_percent"],
                    "flower_counts": flower_counts,
                    "compartment_counts": {
                        side: {
                            lane: sum(
                                1
                                for plant in bed["plants"]
                                if plant["bed_side"] == side and plant["lane"] == lane
                            )
                            for lane in ("left", "right")
                        }
                        for side in ("a", "b")
                    },
                    "bugs_detected": bed["bugs_detected"],
                },
                separators=(",", ":"),
            )
            self._bed_environment_pub.publish(environment_message)

            for compartment in bed["compartments"]:
                compartment_message = String()
                compartment_message.data = json.dumps(
                    {
                        "bed_id": str(bed_id),
                        "side": compartment["bed_side"],
                        "section": compartment["lane"],
                        "color": compartment["color"],
                        "heights_cm": compartment["heights_cm"],
                        "timestamp": iso_now(),
                    },
                    separators=(",", ":"),
                )
                self._plant_health_json_pub.publish(compartment_message)

            for plant in bed["plants"]:
                point = self._point_from_plant(plant)

                analysis_message = PlantAnalysis()
                analysis_message.plant_detected = bool(plant["flower_detected"])
                analysis_message.bugs_detected = bool(plant["bug_detected"])
                analysis_message.fully_grown = bool(plant["ready_for_harvest"])
                analysis_message.height = float(plant["height_cm"])
                analysis_message.color = plant["color"]
                analysis_message.position = point
                analysis_message.message = plant["notes"]
                self._plant_analysis_pub.publish(analysis_message)

                plant_message = self._make_plant_health(bed_id, plant, ros_scan_time)
                self._plant_health_pub.publish(plant_message)

        report_message = String()
        report_message.data = json.dumps(
            {
                "totalBeds": len(self._perception_beds),
                "totalFlowers": report_total_flowers,
                "lastScanTime": iso_now(),
                "nextAction": "Inspect bed 2 for bugs and bed 4 for harvest readiness.",
            },
            separators=(",", ":"),
        )
        self._plant_health_report_pub.publish(report_message)

    def _point_from_plant(self, plant: dict[str, Any]) -> Point:
        point = Point()
        point.x, point.y, point.z = plant["position"]
        return point

    def _make_plant_health(self, bed_id: int, plant: dict[str, Any], scan_time) -> PlantHealth:
        point = self._point_from_plant(plant)
        message = PlantHealth()
        message.flower_id = plant["flower_id"]
        message.bed_id = str(bed_id)
        message.height_cm = float(plant["height_cm"])
        message.color = plant["color"]
        message.health = plant["health"]
        message.growth_stage = plant["growth_stage"]
        message.bug_detected = bool(plant["bug_detected"])
        message.flower_detected = bool(plant["flower_detected"])
        message.ready_for_harvest = bool(plant["ready_for_harvest"])
        message.confidence = float(plant["confidence"])
        message.last_scan_time = scan_time
        message.notes = plant["notes"]
        message.position = point
        return message

    def _on_execute_behavior(self, goal_handle) -> ExecuteBehavior.Result:
        behavior = int(goal_handle.request.behavior.type)
        target_id = str(goal_handle.request.target_id or "").strip()
        behavior_name = self._behavior_name(behavior)
        feedback = ExecuteBehavior.Feedback()
        feedback.current_step = f"test {behavior_name} accepted"
        feedback.progress = 0.25
        goal_handle.publish_feedback(feedback)

        if behavior == BehaviorType.INSPECT_BED:
            bed_id = target_id.split(":", 1)[0].strip() if target_id else "1"
            side = target_id.split(":", 1)[1].strip() if ":" in target_id else "a"
            self._publish_test_scan_for_bed(bed_id, side)
            message = f"test INSPECT_BED completed for {bed_id}:{side}"
        elif behavior == BehaviorType.ARM_TEST:
            message = "test ARM_TEST accepted"
        elif behavior == BehaviorType.NAVIGATE:
            message = "test NAVIGATE accepted"
        else:
            message = f"test {behavior_name} accepted"

        feedback.current_step = message
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = ExecuteBehavior.Result()
        result.success = True
        result.message = message
        return result

    def _publish_test_scan_for_bed(self, bed_id: str, side: str) -> None:
        bed = next(
            (candidate for candidate in self._perception_beds if str(candidate["bed_id"]) == str(bed_id)),
            None,
        )
        if bed is None:
            return
        plants = [plant for plant in bed["plants"] if plant["side"] == ("left" if side == "a" else "right")]
        if not plants:
            plants = list(bed["plants"])
        active_plant = plants[0] if plants else None
        scan = ScanProgress()
        scan.active_bed_id = str(bed["bed_id"])
        scan.active_scan_position_id = f"{bed['bed_id']}:{side}"
        scan.active_flower_id = active_plant["flower_id"] if active_plant else ""
        scan.scan_index = len(plants)
        scan.scan_total = max(1, len(plants))
        scan.detection_status = "test_inspect_complete"
        scan.retry_count = 0
        scan.error = False
        scan.message = f"test inspection completed for bed {bed['bed_id']} side {side}"
        if active_plant:
            scan.latest_plant_health = self._make_plant_health(
                int(bed["bed_id"]),
                active_plant,
                self.get_clock().now().to_msg(),
            )
        self._scan_progress_pub.publish(scan)

    def _behavior_name(self, behavior: int) -> str:
        names = {
            BehaviorType.IDLE: "IDLE",
            BehaviorType.TELEOP: "TELEOP",
            BehaviorType.MAP: "MAP",
            BehaviorType.LOCALIZE: "LOCALIZE",
            BehaviorType.INSPECT_BED: "INSPECT_BED",
            BehaviorType.INSPECT_FLOWER: "INSPECT_FLOWER",
            BehaviorType.HARVEST: "HARVEST",
            BehaviorType.ARM_TEST: "ARM_TEST",
            BehaviorType.NAVIGATE: "NAVIGATE",
        }
        return names.get(behavior, f"behavior_{behavior}")

    def _on_set_robot_mode(
        self,
        request: SetRobotMode.Request,
        response: SetRobotMode.Response,
    ) -> SetRobotMode.Response:
        response.success = True
        response.message = f"test task mode accepted: {request.mode}"
        task = TaskStatus()
        task.current_state = request.mode
        task.active = request.mode not in ("", "IDLE")
        task.error = False
        task.message = response.message
        self._task_status_pub.publish(task)
        return response

    def _on_send_named_arm_pose(
        self,
        request: SendNamedArmPose.Request,
        response: SendNamedArmPose.Response,
    ) -> SendNamedArmPose.Response:
        response.accepted = True
        response.message = f"test arm pose accepted: {request.pose_name}"
        return response

    def _on_start_mapping(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success = True
        response.message = "test mapping started"
        return response

    def _on_done_mapping(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success = True
        response.message = "test mapping finalized"
        return response

    def _on_save_safe_map(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success = True
        response.message = "test safe map save received"
        return response


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish test-only SimBioSys UI data over ROS 2 topics."
    )
    parser.add_argument(
        "--map-period",
        type=positive_float,
        default=20.0,
        help="Seconds between /map publishes.",
    )
    parser.add_argument(
        "--bed-period",
        type=positive_float,
        default=5.0,
        help="Seconds between bed environment and plant report publishes.",
    )
    parser.add_argument(
        "--odom-period",
        type=positive_float,
        default=0.2,
        help="Seconds between odometry publishes.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish one set of messages and exit.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = UiTestDataPublisher(
        map_period=args.map_period,
        bed_period=args.bed_period,
        odom_period=args.odom_period,
        once=args.once,
    )

    try:
        if args.once:
            rclpy.spin_once(node, timeout_sec=0.5)
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

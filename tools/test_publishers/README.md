# SimBioSys UI Test Data Publisher

This is a standalone, test-only ROS 2 publisher for exercising the SimBioSys UI
with artificial data. It is not UI dummy mode, and it does not add fallback or
mock data to `src/simbiosys_ui`. The UI remains purely data-driven and receives
messages through normal ROS topics.

## Run

Terminal 1:

```bash
cd /home/mark/MDP
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run simbiosys_ui ui_node
```

Terminal 2:

```bash
cd /home/mark/MDP
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/test_publishers/ui_test_data_publisher.py
```

Useful options:

```bash
python3 tools/test_publishers/ui_test_data_publisher.py --map-period 10
python3 tools/test_publishers/ui_test_data_publisher.py --bed-period 2 --odom-period 0.1
python3 tools/test_publishers/ui_test_data_publisher.py --once
```

## Published Topics

| Topic | Type | Data |
| --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Greenhouse map with outer walls, three rectangular beds, obstacles, false-scan blobs, and free space. |
| `/mirte_base_controller/odom` | `nav_msgs/msg/Odometry` | Slowly moving robot pose with `frame_id: odom` and `child_frame_id: base_link`. |
| `/mapping/artifact_candidates` | `std_msgs/msg/String` | JSON artifact candidates named `artifact_1` through `artifact_5`; all are unclassified. |
| `/bed_environment` | `std_msgs/msg/String` | JSON bed environment records for Bed A, Bed B, and Bed C. |
| `/simbiosys/bed_observation` | `std_msgs/msg/String` | Same JSON bed environment records, published here too to test which topic the UI uses. |
| `/plant_health_report` | `std_msgs/msg/String` | JSON combined report containing all beds and a timestamp. |

Defaults:

- `/map`: every 20 seconds, configurable with `--map-period`
- `/mapping/artifact_candidates`: on startup and every 20 seconds
- `/mirte_base_controller/odom`: every 0.2 seconds, configurable with `--odom-period`
- Bed topics and `/plant_health_report`: every 5 seconds, configurable with `--bed-period`

## Test-Only Services

These services are not implemented by the real mapping package in this
repository. They are provided here only so the UI can exercise its configured
mapping workflow without adding dummy logic to `src/simbiosys_ui`.

| Service | Type | Response |
| --- | --- | --- |
| `/mapping/start` | `std_srvs/srv/Trigger` | `success: true`, `message: "test mapping started"` |
| `/mapping/done` | `std_srvs/srv/Trigger` | `success: true`, `message: "test mapping finalized"` and republishes artifact candidates |
| `/mapping/save_safe_map` | `std_srvs/srv/Trigger` | `success: true`, `message: "test safe map save received"` |

## Mapping Schema

The real `simbiosys_mapping` package currently provides `/map`,
`simbiosys/mapping_status`, and `/getmap_node/save_map`. It does not currently
implement `/mapping/artifact_candidates`; see
`tools/test_publishers/mapping_schema_comparison.md`.

The test publisher sends the intended JSON integration schema on
`/mapping/artifact_candidates`:

- top-level `timestamp`
- top-level `frame_id`
- top-level `candidates` array
- candidate `id` as `artifact_1`, `artifact_2`, etc.
- candidate `candidate_type: "unclassified"`
- candidate `source: "slam"`
- optional `confidence`
- nested `geometry.type`
- nested geometry values such as polygon `points`, rectangle `pose` and
  `size`, or circle `center` and `radius`

The test publisher does not send `suggested_class`, `class`, or semantic
`kind` values. Classification is manual in the UI. During Save Safe Map,
artifacts left unclassified are treated as `false_scan` and excluded from the
safe map, along with artifacts manually classified as `false_scan`.

## Topic Checks

```bash
ros2 topic list -t | grep -E "mapping|artifact|candidate|map|bed|environment"
ros2 service list -t | grep -E "mapping|map|safe|start|done|save"
ros2 topic echo /mapping/artifact_candidates --once --field data
ros2 topic echo /bed_environment
ros2 topic echo /map --once
ros2 service call /mapping/start std_srvs/srv/Trigger {}
ros2 service call /mapping/done std_srvs/srv/Trigger {}
ros2 service call /mapping/save_safe_map std_srvs/srv/Trigger {}
```

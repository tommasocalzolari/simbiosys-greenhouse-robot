# SimBioSys UI

Operator UI for the ROS 2 Humble SimBioSys greenhouse robot.

The UI shows real ROS data when the matching topic, service, action, or project
file is available. Missing backends are shown as waiting or unavailable instead
of being silently faked.

## Start

From the workspace root:

```bash
cd /home/mark/MDP
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run simbiosys_ui ui_node
```

Open:

```text
http://localhost:8080
```

The web server listens on `0.0.0.0:8080`, so another trusted device on the same
local network can open:

```text
http://<LAPTOP_LAN_IP>:8080
```

Find the laptop IP with:

```bash
hostname -I
```

Do not expose this UI directly to the public internet or forward the port on a
router. See `docs/network_access.md` for network notes.

## Main Views

### Dashboard

The dashboard contains the main robot overview:

- global `STOP` / `START` UI command pause
- battery status from `/io/power/power_watcher`
- current task/status information
- live SLAM map from `/map`
- start pose, navigation goal, cancel navigation, and go home controls
- plant bed cards with bed environment, bed health, flower markers, and selected
  flower details
- mapping workflow controls and artifact review when those backends are present

The dashboard map is updated at the configured UI refresh period
`mapUpdatePeriodSec`, currently `10` seconds. This only controls how often the
web UI sends the latest received map image to the browser.

### Teleop / Camera

The Teleop / Camera page contains manual base movement and camera viewing.
Mapping controls are intentionally not duplicated here; the map workflow lives on
the dashboard.

Teleop supports on-screen buttons and keyboard control:

- `W`: forward
- `S`: backward
- `A`: strafe left
- `D`: strafe right
- `Q`: rotate counter-clockwise
- `E`: rotate clockwise
- `Space` or `Escape`: stop / zero velocity

Movement keys combine into one Twist command. Opposite keys cancel on their
axis. Releasing keys/buttons, hiding the page, unloading the page, navigating
away from Teleop, disconnecting, or stopping the UI sends zero velocity.

Speed modes:

- slow: `0.50 m/s`, `0.8 rad/s`
- normal: `0.75 m/s`, `1.4 rad/s`
- fast: `1.00 m/s`, `2.0 rad/s`

The global red `STOP` / green `START` button is a UI-side command pause. It
publishes zero Twist and blocks UI movement/navigation commands, but it is not a
hardware emergency stop.

### Arm

The arm controls call `simbiosys/send_named_arm_pose`
(`simbiosys_interfaces/srv/SendNamedArmPose`) for named poses such as inspect
and home, when the arm backend is available.

## Camera Data

The UI uses compressed image topics only:

- base camera: `/camera/color/image_raw/compressed`
- wrist/gripper camera: `/gripper_camera/image_raw/compressed`

Raw image topics are configured as `null` and are not subscribed to by the UI.
Camera subscriptions are lazy:

- the base camera is only subscribed while Teleop is open and the base camera is
  selected
- the wrist camera is only subscribed while Teleop is open and the wrist camera
  is selected

The UI does not cap camera FPS in the current configuration. If bandwidth must be
reduced further, cap the camera publisher or compression transport on the robot
side so the data is never sent over the network.

## Navigation

Dashboard navigation uses:

- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- `/plan` (`nav_msgs/msg/Path`)
- `/amcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `/mirte_base_controller/odom` (`nav_msgs/msg/Odometry`)

The Go Home button sends the configured `homePose` from
`simbiosys_ui/config/rosTopics.json`.

## Behavior And Plant Beds

The UI is ready for bed identity to come from the behavior package. It subscribes
to:

- `simbiosys/scan_progress` (`simbiosys_interfaces/msg/ScanProgress`)
- `simbiosys/harvest_status` (`simbiosys_interfaces/msg/HarvestStatus`)

`ScanProgress.active_bed_id` and `HarvestStatus.active_bed_id` create or update
bed cards in the UI. If behavior sends a bed-side id such as `bed_1:a`, the UI
normalizes that to `bed_1` for the bed card.

Plant and bed health data can come from:

- `simbiosys/plant_health` (`simbiosys_interfaces/msg/PlantHealth`)
- `simbiosys/bed_observation` (`simbiosys_interfaces/msg/BedObservation`)
- `/plant_health` (`std_msgs/msg/String`)
- `/plant_health_report` (`std_msgs/msg/String`)
- `/simbiosys/flower_counts` (`std_msgs/msg/String`)
- `/bed_environment` (`std_msgs/msg/String`)

The expected UI behavior is:

- each bed card shows CO2, humidity, and bug detection status when data exists
- bed borders indicate health/warning/error state
- flower markers are shown inside the bed cards
- clicking a flower marker shows that flower's details in a separate flower info
  panel below the bed cards

`INSPECT_BED` currently goes through `simbiosys/execute_behavior`
(`simbiosys_interfaces/action/ExecuteBehavior`). The behavior implementation
expects `target_id=<bed_id>:<side>` for a real bed-side scan.

## Mapping

Mapping information is shown on the dashboard.

Current mapping-related interfaces used by the UI:

- `/map` (`nav_msgs/msg/OccupancyGrid`)
- `simbiosys/mapping_status` (`simbiosys_interfaces/msg/MappingStatus`)
- `/mapping/start` (`std_srvs/srv/Trigger`)
- `/mapping/done` (`std_srvs/srv/Trigger`)
- `/mapping/save_safe_map` (`std_srvs/srv/Trigger`)
- `/mapping/artifact_candidates` (`std_msgs/msg/String`)

The UI does not create mapping artifacts by itself. Artifact candidates, safe
map saving, and mapping status must come from the mapping backend or from the
test publisher.

For manual UI testing, the repository contains test publisher tooling under:

```text
tools/test_publishers/
```

## Configuration

Edit the topic configuration here:

```text
src/simbiosys_ui/simbiosys_ui/config/rosTopics.json
```

Main configured topics and backends:

- `cmdVel`: `/mirte_base_controller/cmd_vel`
- `baseCameraCompressed`: `/camera/color/image_raw/compressed`
- `armCameraCompressed`: `/gripper_camera/image_raw/compressed`
- `liveMap`: `/map`
- `mapUpdatePeriodSec`: `10`
- `mappingStatus`: `simbiosys/mapping_status`
- `taskStatus`: `simbiosys/task_status`
- `scanProgress`: `simbiosys/scan_progress`
- `harvestStatus`: `simbiosys/harvest_status`
- `setTaskMode`: `simbiosys/set_robot_mode`
- `executeBehavior`: `simbiosys/execute_behavior`
- `sendNamedArmPose`: `simbiosys/send_named_arm_pose`
- `battery`: `/io/power/power_watcher`

After changing package data such as `rosTopics.json`, rebuild or reinstall the
package so the installed copy is updated.

## Useful Commands

Build only the UI package:

```bash
cd /home/mark/MDP
colcon build --packages-select simbiosys_ui --symlink-install
source install/setup.bash
```

Run the UI:

```bash
ros2 run simbiosys_ui ui_node
```

Run the standalone teleop interface node, if needed:

```bash
ros2 run simbiosys_ui teleop_interface_node
```

Static syntax checks:

```bash
python3 -m py_compile src/simbiosys_ui/simbiosys_ui/ui_node.py
python3 -m json.tool src/simbiosys_ui/simbiosys_ui/config/rosTopics.json >/dev/null
```

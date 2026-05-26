# UI Topic Contract

Config file:

```text
simbiosys_ui/config/rosTopics.json
```

## Real Discovered Interfaces

| Purpose | Name | Type | Owner | Used by UI | Notes |
| --- | --- | --- | --- | --- | --- |
| Teleop velocity | `/mirte_base_controller/cmd_vel` | `geometry_msgs/msg/Twist` | MIRTE base controller | yes | UI publisher uses queue depth 1 by default and publishes zero Twist on STOP, release, hide, unload, timeout, and shutdown. |
| Base/front camera compressed | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | MIRTE simulation/camera driver | yes, only on Teleop page when selected | UI subscribes only to the selected compressed camera stream and caps UI updates to 1 FPS by default. |
| Arm/gripper camera compressed | `/gripper_camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | MIRTE robot topic config | yes, only on Teleop page when selected | Camera selector enables arm camera only when a compressed topic is configured; UI updates are capped to 1 FPS by default. |
| Live map | `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` / Nav2 map server | yes | Dashboard target selection and Teleop SLAM panel use only real map messages. |
| Odometry pose | `/mirte_base_controller/odom` | `nav_msgs/msg/Odometry` | MIRTE base controller | yes | Robot marker is drawn only on the Teleop SLAM map. |
| Localized pose | `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Nav2 AMCL | yes | Overrides odometry pose when received. |
| Mapping status | `simbiosys/mapping_status` | `simbiosys_interfaces/msg/MappingStatus` | `simbiosys_mapping/mapping_status_node.py` | yes | Status-only. |
| Artifact candidates | `/mapping/artifact_candidates` | `std_msgs/msg/String` JSON | intended mapping/test integration publisher | yes | Real mapping package does not implement this yet. UI parses only received JSON candidates. Supports flat geometry fields and nested `geometry` objects. |
| Task status | `simbiosys/task_status` | `simbiosys_interfaces/msg/TaskStatus` | `simbiosys_behavior/mission_manager_node.py` | yes | Shows current backend state. |
| Set task mode | `simbiosys/set_robot_mode` | `simbiosys_interfaces/srv/SetRobotMode` | `simbiosys_behavior/mission_manager_node.py` | yes | Dashboard maps `harvest` to `HARVESTING` and `scanning` to `SCANNING`. |
| Behavior command | `simbiosys/execute_behavior` | `simbiosys_interfaces/action/ExecuteBehavior` | `simbiosys_behavior/mission_manager_node.py` | yes | Dashboard sends `NAVIGATE`; Take Control sends `TELEOP`; Release/STOP sends `IDLE` when available. |
| Named arm pose | `simbiosys/send_named_arm_pose` | `simbiosys_interfaces/srv/SendNamedArmPose` | `simbiosys_arm/named_joint_pose_node.py` | yes | Arm Operations uses existing names: `home`, `camera_forward`, `camera_down`, `inspect`, `stow`. |
| Typed plant record | `simbiosys/plant_health` | `simbiosys_interfaces/msg/PlantHealth` | `simbiosys_perception/plant_analysis_node.py` | yes | UI displays only selected useful fields: ID, color, growth stage, ready flag, scan age. |
| Legacy plant record | `/plant_health` | `std_msgs/msg/String` JSON | legacy/UI integration | yes | Must include a real `flower_id`; otherwise ignored. |
| Plant report | `/plant_health_report` | `std_msgs/msg/String` JSON/text | future/report backend | yes | Optional summary source. |
| Bed observation | `simbiosys/bed_observation` | `simbiosys_interfaces/msg/BedObservation` | `simbiosys_perception/apriltag_detection_node.py` | limited | Gives real bed IDs/visibility only; no CO2/humidity/bugs fields exist. |
| Battery | `/io/power/power_watcher` | `sensor_msgs/msg/BatteryState` | MIRTE telemetrix INA226 power watcher | yes | Shows percentage from the same power watcher that drives the robot battery indicator; UI value updates at most once per minute. |
| Raw map save | `/getmap_node/save_map` | `std_srvs/srv/Trigger` | `simbiosys_mapping/getmap_node.py` | no | Saves raw map only; not a reviewed safe-map backend. |

## Missing Required Interfaces

| Purpose | Config key / suggested name | Current UI behavior |
| --- | --- | --- |
| Dedicated full motor/autonomy stop | `takeControl` or `/ui/take_control` | Missing. STOP always publishes zero Twist and disables UI commands; behavior `IDLE` is requested when available. |
| Dedicated release-control/resume policy | TBD | Missing. Release Control publishes zero Twist, disables teleop, and requests behavior `IDLE` when available. |
| Start mapping | `startMapping` or `/mapping/start` | Disabled unless configured Trigger service is available. Real mapping package does not provide this; test publisher does. |
| Done/finalize mapping backend | `doneMapping` or `/mapping/done` | Enabled after a map is received. Calls backend when available; otherwise UI can freeze latest received real map locally. Test publisher provides this Trigger service. |
| Artifact classification backend | `classifyArtifact` or `/mapping/classify_artifact` | Missing. UI can locally classify received real candidates for review preview only. |
| Reviewed safe map save | `saveSafeMap` or `/mapping/save_safe_map` | Disabled unless reviewed candidates, local classifications, and configured backend are available. Real mapping package does not provide this; test publisher does. |
| Safe map output | `safeMapOutput` or `/mapping/safe_map` | Not displayed until real output exists. |
| Bed CO2 | TBD | Bed card shows unavailable. |
| Bed humidity | TBD | Bed card shows unavailable. |
| Bed-level bug status | TBD | Bed card shows unavailable. |

## Teleop Safety

The global STOP/START button is a UI command pause, not a hardware emergency
stop. STOP immediately publishes zero Twist, requests behavior `IDLE` when the
behavior action is available, disables UI movement/navigation/arm commands, and
clears Take Control. START re-enables the UI safety state but does not enable
movement; Take Control is still required.

Robot movement commands require all of:

- global safety state is START/enabled
- Take Control is active
- Robot Operations mode is active

Keyboard mapping:

- `W`: positive `linear.x`
- `S`: negative `linear.x`
- `A`: positive `linear.y`
- `D`: negative `linear.y`
- `Q`: positive `angular.z`
- `E`: negative `angular.z`

Speed modes:

- slow: `0.50 m/s`, `0.8 rad/s`
- normal: `0.75 m/s`, `1.4 rad/s`
- fast: `1.00 m/s`, `2.0 rad/s`

Opposite keys cancel on their axis. Diagonal planar movement is normalized so
the selected linear speed is not exceeded.

## Artifact Candidate JSON

The configured artifact candidate topic is `/mapping/artifact_candidates` with
type `std_msgs/msg/String`. The payload may be either a JSON object with a
`candidates` array or a JSON array of candidate objects.

Real mapping artifact candidate interface not implemented yet. The standalone
test publisher uses the currently documented intended interface:

- top-level `timestamp`
- top-level `frame_id`
- top-level `candidates` array
- candidate `id`, `kind`, `suggested_class`, `source`, `confidence`
- nested `geometry.type`
- nested rectangle geometry as `geometry.x`, `geometry.y`, `geometry.width`,
  and `geometry.height`
- nested polygon geometry as `geometry.points`

The UI supports the older flat schema:

- `geometry_type`
- `points`
- `pose`
- `size`
- `suggested_class`

It also supports the nested schema currently published by the test publisher:

- `kind`
- `geometry.type`
- `geometry.points`
- `geometry.pose`
- `geometry.size`
- `geometry.x`, `geometry.y`, `geometry.width`, `geometry.height`
- `geometry.radius`
- `geometry.center`
- `geometry.start`
- `geometry.end`

Received candidates are normalized internally to `id`, `classHint`,
`geometryType`, `points`, `pose`, `size`, `radius`, `center`, `start`, `end`,
and `raw`. The original raw candidate is preserved for debugging. The UI does
not generate artifact candidates if the topic is absent or empty.

## Mapping Workflow Diagnostics

The Mapping Workflow panel shows:

- map topic connected/waiting and last received timestamp
- artifact candidate count
- Start Mapping backend availability
- Save Safe Map backend availability
- selected candidate ID
- review mode active/inactive

Disabled mapping buttons explain the current blocker in the workflow message
and button title, for example `Start mapping service unavailable`, `Save safe
map backend unavailable`, `No map received yet`, or `No artifact candidates
received`.

## Verification Commands

```bash
ros2 topic list -t | grep -E "mapping|artifact|candidate|map|bed|environment"
ros2 service list -t | grep -E "mapping|map|safe|start|done|save"
ros2 topic echo /mapping/artifact_candidates --once --field data
ros2 topic echo /map --once
ros2 service call /mapping/start std_srvs/srv/Trigger {}
ros2 service call /mapping/done std_srvs/srv/Trigger {}
ros2 service call /mapping/save_safe_map std_srvs/srv/Trigger {}
```

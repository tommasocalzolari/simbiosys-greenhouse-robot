# SimBioSys UI

Embedded ROS 2 Humble operator UI for the MIRTE Master greenhouse robot.

The UI is served by `simbiosys_ui.ui_node`. It shows real ROS data only: if a
topic, service, action, or backend is unavailable, the UI shows a waiting or
unavailable state instead of inventing data.

## Run

From the workspace root:

```bash
cd /home/mark/MDP
pixi run bash -lc 'source install/setup.bash && python -m simbiosys_ui.ui_node'
```

Open:

```text
http://localhost:8080
```

The UI binds to `0.0.0.0` by default, so devices on the same trusted local
network can open:

```text
http://<LAPTOP_LAN_IP>:8080
```

Do not expose the UI directly to the public internet. Use a VPN such as
Tailscale or WireGuard for off-network access.

Optional environment variables:

```bash
SIMBIOSYS_UI_HOST=127.0.0.1
SIMBIOSYS_UI_PORT=8090
```

## Build

```bash
cd /home/mark/MDP
pixi run colcon build --packages-select simbiosys_ui --symlink-install
source install/setup.bash
```

## Main Screens

### Dashboard

The dashboard contains:

- global `STOP` / `START` UI command pause
- connection and battery status
- map display
- robot pose marker
- Nav2 `/plan` overlay
- selected map target marker
- `Set Start Pose`
- `Navigate`
- `Cancel Navigation`
- `Go Home`
- task mode selector
- scan summary
- bed overview
- flower info panel

`Go Home` navigates to the configured `homePose` in
`simbiosys_ui/config/rosTopics.json`. The default is:

```json
{"x": 0.0, "y": 0.0, "yaw": 0.0}
```

### Teleop / Camera

The Teleop / Camera page contains only camera and operation controls. Mapping
widgets are intentionally not shown here anymore; the map is on the dashboard.

Camera subscriptions are lazy:

- no camera topic is subscribed while the dashboard is active
- only the selected compressed camera topic is subscribed while Teleop / Camera
  is active
- switching camera destroys the old camera subscription

The UI uses compressed camera topics:

- base: `/camera/color/image_raw/compressed`
- arm/wrist: `/gripper_camera/image_raw/compressed`

The UI no longer applies a local camera FPS cap. If bandwidth needs to be
reduced, cap the camera publisher/driver on the robot side.

## STOP / START

The red `STOP` / green `START` button is a UI command pause, not a hardware
emergency stop.

When stopped, the UI:

- publishes a zero `geometry_msgs/msg/Twist`
- clears active teleop controls
- disables UI teleop, navigation, mapping, task, and arm commands
- requests `BehaviorType.IDLE` if the behavior action backend is available

Other packages can still publish directly to hardware topics unless a separate
robot-side safety gate exists.

## Teleop

Before driving, press `Take Control`.

Keyboard controls:

- `W`: forward
- `S`: backward
- `A`: strafe left
- `D`: strafe right
- `Q`: rotate counter-clockwise
- `E`: rotate clockwise
- `Space` or `Escape`: stop

On-screen controls support the same movement directions. Movement keys/buttons
combine into one Twist command. Opposite directions cancel on their axis.

The UI sends zero velocity when:

- controls are released
- the page is hidden
- the browser leaves the page
- the user returns to the dashboard
- input becomes stale
- `STOP` is pressed
- the UI node shuts down

Speed modes:

- slow: `0.50 m/s`, `0.8 rad/s`
- normal: `0.75 m/s`, `1.4 rad/s`
- fast: `1.00 m/s`, `2.0 rad/s`

The teleop publisher queue depth defaults to `1`.

## Navigation and Localization

The dashboard map uses the `map` frame.

Subscriptions:

- `/map` (`nav_msgs/msg/OccupancyGrid`)
- `/amcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `/mirte_base_controller/odom` (`nav_msgs/msg/Odometry`)
- `/plan` (`nav_msgs/msg/Path`)

Map QoS uses `TRANSIENT_LOCAL` durability so the UI can receive the latest map
even if it was published before the UI subscribed.

Controls:

- `Set Start Pose`: publishes the selected map point to `/initialpose`
  (`geometry_msgs/msg/PoseWithCovarianceStamped`) several times.
- `Navigate`: first tries `/navigate_to_pose`
  (`nav2_msgs/action/NavigateToPose`), then falls back to `/goal_pose`
  (`geometry_msgs/msg/PoseStamped`), then falls back to
  `simbiosys/execute_behavior` with `BehaviorType.NAVIGATE`.
- `Cancel Navigation`: cancels the active Nav2 goal when the UI owns one.
- `Go Home`: sends a navigation goal to the configured `homePose`.

Navigation availability is displayed from the available backend:

- Nav2 action `/navigate_to_pose`
- `/goal_pose` publisher
- SimBioSys behavior action `simbiosys/execute_behavior`

## Arm Operations

Arm pose buttons are shown under Teleop / Camera after selecting `Arm
Operations`.

The UI calls:

```text
simbiosys/send_named_arm_pose
simbiosys_interfaces/srv/SendNamedArmPose
```

Available pose names:

- `home`
- `camera_forward`
- `camera_down`
- `inspect`
- `stow`

Start the arm pose backend separately if the full system launch does not start
it:

```bash
cd /home/mark/MDP
pixi run bash -lc 'source install/setup.bash && python -m simbiosys_arm.named_joint_pose_node'
```

Do not press arm pose buttons while connected to real hardware unless movement
is intended and safe.

## Beds, Flowers, and Plant Health

The dashboard bed overview is driven by perception and test data.

Typed perception topics:

- `simbiosys/bed_observation`
  (`simbiosys_interfaces/msg/BedObservation`)
- `simbiosys/plant_health`
  (`simbiosys_interfaces/msg/PlantHealth`)

Additional current UI topics:

- `/simbiosys/flower_counts` (`std_msgs/msg/String` JSON)
- `/bed_environment` (`std_msgs/msg/String` JSON)
- `/plant_health` (`std_msgs/msg/String` JSON fallback)
- `/plant_health_report` (`std_msgs/msg/String` JSON fallback)

Bed cards show:

- CO2
- humidity
- bug detection
- flower circles in two rows
- `Inspect Bed`

Bed border status:

- green: normal
- yellow: CO2 or humidity warning
- red: bugs, both CO2 and humidity warning, or extreme CO2/humidity values

Flower circles are labeled per bed in display order, for example `1a`, `1b`,
`1c`. Clicking a flower updates the separate `Flower Info` panel under the bed
overview.

`Inspect Bed` sends `BehaviorType.INSPECT_BED` through
`simbiosys/execute_behavior` with `target_id=<bed_id>`.

## Battery

Battery percentage is read from:

```text
/io/power/power_watcher
sensor_msgs/msg/BatteryState
```

The UI updates the displayed percentage at most once per minute.

## Mapping and Artifacts

The dashboard can display `/map`, but mapping workflow controls are no longer
part of the visible Teleop / Camera screen.

The backend still contains compatibility routes for the older mapping workflow:

- `/api/mapping/start`
- `/api/mapping/done`
- `/api/mapping/save_safe_map`

Configured mapping interfaces:

- `/mapping/start` (`std_srvs/srv/Trigger`)
- `/mapping/done` (`std_srvs/srv/Trigger`)
- `/mapping/save_safe_map` (`std_srvs/srv/Trigger`)
- `/mapping/artifact_candidates` (`std_msgs/msg/String` JSON)
- `simbiosys/mapping_status`
  (`simbiosys_interfaces/msg/MappingStatus`)

The real `simbiosys_mapping` package currently provides real map and status
data, but artifact candidate generation and safe-map services may require a
test/demo publisher or future backend implementation.

## Test Data Publisher

For local UI testing without the real perception pipeline:

```bash
cd /home/mark/MDP
pixi run bash -lc 'source install/setup.bash && python tools/test_publishers/ui_test_data_publisher.py --bed-period 2'
```

Then run the UI in another terminal:

```bash
cd /home/mark/MDP
pixi run bash -lc 'source install/setup.bash && python -m simbiosys_ui.ui_node'
```

The test publisher simulates current perception-shaped bed, flower, plant
health, flower count, and bed environment data.

## Configuration

Edit:

```text
src/simbiosys_ui/simbiosys_ui/config/rosTopics.json
```

Main configurable entries:

- `cmdVel`
- `baseCameraCompressed`
- `armCameraCompressed`
- `liveMap`
- `mapUpdatePeriodSec`
- `initialPose`
- `goalPose`
- `navigateToPose`
- `navPlan`
- `homePose`
- `mappingStatus`
- `taskStatus`
- `setTaskMode`
- `executeBehavior`
- `sendNamedArmPose`
- `odom`
- `amclPose`
- `typedPlantHealth`
- `flowerCounts`
- `bedObservation`
- `bedEnvironment`
- `battery`

Launch parameters can override selected runtime values:

- `web_host`
- `web_port`
- `cmd_vel_topic`
- `teleop_queue_depth`
- `image_topic`
- `compressed_image_topic`
- `live_map_topic`

`camera_max_fps` may still appear in older launch files for compatibility, but
the current UI node no longer uses it to drop camera frames.

## HTTP API

The browser talks to the local UI backend through these routes:

- `GET /api/status`
- `GET /stream.mjpg`
- `POST /api/view`
- `POST /api/safety/toggle`
- `POST /api/take_control`
- `POST /api/teleop`
- `POST /api/camera/select`
- `POST /api/navigation/initial_pose`
- `POST /api/navigation/goal`
- `POST /api/navigation/cancel`
- `POST /api/navigation/home`
- `POST /api/task_mode`
- `POST /api/arm/pose`
- `POST /api/bed/inspect`
- `POST /api/mapping/start`
- `POST /api/mapping/done`
- `POST /api/mapping/save_safe_map`

These routes are intended for the embedded UI, not for public internet access.

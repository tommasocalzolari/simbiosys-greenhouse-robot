# SimBioSys UI

Embedded operator UI for the ROS 2 Humble MIRTE Master greenhouse robot.

## Start the UI

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

The UI also listens on the trusted local network by default. To open it from a phone/tablet/laptop on the same Wi-Fi, find the laptop IP with `hostname -I` and open:

```text
http://<LAPTOP_LAN_IP>:8080
```

See `docs/network_access.md` for setup, test, firewall, and rosbridge notes. Do not expose the UI to the public internet or use router port forwarding.

The first screen is the dashboard with a 2D greenhouse digital twin map, flower-level plant health, bed summaries, and a concise report. Use the **Teleop / Camera** button to open the separate camera and teleoperation page.

The dummy greenhouse map contains exactly three beds:

- Bed A: `A1` through `A20`
- Bed B: `B1` through `B18`
- Bed C: `C1` through `C22`

Each flower has its own height, color, health, growth stage, bug status, harvest readiness, confidence, scan time, and notes. Bed panels are summaries only.

## Teleop Controls

The Teleop / Camera page supports both on-screen buttons and keyboard control:

- `W`: forward
- `S`: backward
- `A`: strafe left
- `D`: strafe right
- `Q`: rotate counter-clockwise
- `E`: rotate clockwise
- `Space` or `Escape`: stop / zero velocity

Keyboard control is active only on the Teleop / Camera page and is ignored while typing in inputs or using the speed selector. Movement keys combine into one Twist command, so `W + A` moves forward-left and opposite keys such as `W + S`, `A + D`, or `Q + E` cancel on that axis. Releasing keys/buttons, hiding the page, unloading the page, or navigating back to the dashboard sends zero velocity when no movement remains.

## Dummy Mode

Dummy mode is enabled by default in:

```text
simbiosys_ui/config/rosTopics.json
```

Set `dummyMode` to `false`, or override it at launch:

```bash
ros2 run simbiosys_ui ui_node --ros-args -p dummy_mode:=false
```

Dummy mode keeps the UI useful without Gazebo, rosbridge, camera frames, `/map`, or plant-health publishers. The current MIRTE Gazebo simulation does not publish `/map`, so the dummy 2D greenhouse map remains the default.

## Configure Topics

Edit:

```text
simbiosys_ui/config/rosTopics.json
```

The UI currently uses configurable names for `cmdVel`, `cameraCompressed`, `cameraRaw`, `map`, `odom`, `amclPose`, `plantHealth`, `plantHealthReport`, `flowerDetections`, `inspectBedCommand`, and `battery`.

Current MIRTE Gazebo simulation topics:

- teleop: `/mirte_base_controller/cmd_vel_unstamped`
- camera compressed: `/camera/image_raw/compressed`
- camera raw fallback: `/camera/image_raw`
- odom: `/mirte_base_controller/odom`
- map: `/map` is configured for future use but is not available in the current simulation, so the dummy map is used

Operation mode is intentionally not part of this UI because the robot LED handles it.

## ROS Connection

The current UI runs as a ROS node and publishes/subscribes directly with `rclpy`. The browser uses relative API and camera URLs, so remote clients connect back to the laptop that served the page. The config still includes `rosbridgeUrl` for future browser-native integration, defaulting to:

```text
ws://localhost:9090
```

When `/api/status` is requested remotely, localhost rosbridge defaults are reported with the current page hostname, such as `ws://192.168.1.50:9090`, so remote clients do not try to connect to their own localhost.

To start rosbridge when installed:

```bash
source /opt/ros/humble/setup.bash
source /home/mark/MDP/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

If that launch file is unavailable, install `ros-humble-rosbridge-server`.

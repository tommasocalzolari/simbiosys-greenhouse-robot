# SimBioSys UI

Embedded operator UI for the ROS 2 Humble MIRTE Master greenhouse robot.

The UI displays only real data received from ROS topics or stored project files.
When a topic, service, action, or project file is unavailable, the UI shows a
waiting or unavailable state.

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

The UI also listens on the trusted local network by default. To open it from a
phone/tablet/laptop on the same Wi-Fi, find the laptop IP with `hostname -I` and
open:

```text
http://<LAPTOP_LAN_IP>:8080
```

See `docs/network_access.md` for setup, test, firewall, and rosbridge notes. Do
not expose the UI to the public internet or use router port forwarding.

## Teleop Controls

The global red `STOP` / green `START` button is a UI command pause. It
publishes zero Twist and disables UI movement/navigation commands, but it is not
a hardware emergency stop.

The Teleop / Camera page supports both on-screen buttons and keyboard control:

- `W`: forward
- `S`: backward
- `A`: strafe left
- `D`: strafe right
- `Q`: rotate counter-clockwise
- `E`: rotate clockwise
- `Space` or `Escape`: stop / zero velocity

Movement keys combine into one Twist command. Opposite keys cancel on their
axis. Releasing keys/buttons, hiding the page, unloading the page, navigating
back to the dashboard, disconnecting, or stopping the UI sends zero velocity.

Speed settings are:

- slow: `0.50 m/s`, `0.8 rad/s`
- normal: `0.75 m/s`, `1.4 rad/s`
- fast: `1.00 m/s`, `2.0 rad/s`

## Mapping

The Teleop / Camera page includes camera, teleop, live map, and mapping workflow
panels. The live map panel subscribes to the configured real
`nav_msgs/msg/OccupancyGrid` topic.

Current real mapping interfaces discovered in the repository:

- `/map` (`nav_msgs/msg/OccupancyGrid`)
- `simbiosys/mapping_status` (`simbiosys_interfaces/msg/MappingStatus`)
- `/getmap_node/save_map` (`std_srvs/srv/Trigger`) for raw map saving from
  `simbiosys_mapping/getmap_node.py`

Real mapping artifact candidate interface not implemented yet. The UI does not
synthesize missing mapping backend features. Start mapping, artifact review,
classification backend calls, and safe-map save stay unavailable until real
interfaces are implemented or the standalone test publisher is running.

For UI workflow testing only, `tools/test_publishers/ui_test_data_publisher.py`
publishes `/mapping/artifact_candidates` and provides `/mapping/start`,
`/mapping/done`, and `/mapping/save_safe_map` as `std_srvs/srv/Trigger`
services. See `tools/test_publishers/mapping_schema_comparison.md` for the
schema comparison.

Dashboard map-position navigation uses the real `simbiosys/execute_behavior`
action when the mission manager is running. The dashboard task selector uses the
real `simbiosys/set_robot_mode` service for `HARVESTING` and `SCANNING` when it
is available.

## Configure Topics

Edit:

```text
simbiosys_ui/config/rosTopics.json
```

Robot LED operation mode remains outside this UI. The Dashboard task selector is
only an operator task request for harvest/scanning workflows.

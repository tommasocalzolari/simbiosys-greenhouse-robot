# UI ROS Connection

The SimBioSys UI currently runs as a ROS 2 Python node, so it connects to ROS through `rclpy` publishers and subscriptions. It does not require rosbridge for the embedded dashboard to run.

The topic config still keeps a `rosbridgeUrl` field for future browser-native ROS integration:

```json
{
  "rosbridgeUrl": "ws://localhost:9090"
}
```

When `/api/status` is requested from another device and this value points at `localhost` or `127.0.0.1`, the UI reports a rosbridge URL using the current page host instead. For example, a browser opened at `http://192.168.1.50:8080` receives `ws://192.168.1.50:9090`. This avoids making a remote browser try to connect to its own `localhost`.

## Start rosbridge

If browser-native ROS access is added later, start rosbridge with:

```bash
source /opt/ros/humble/setup.bash
source /home/mark/MDP/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Rosbridge must listen on the laptop, and port `9090` must be reachable from devices on the same trusted local network. Do not expose rosbridge to the public internet.

If the package is missing:

```bash
sudo apt install ros-humble-rosbridge-server
```

## Missing Topics

The UI is defensive when topics are missing:

- Camera page shows a placeholder until frames arrive.
- Dashboard uses dummy greenhouse beds when `/map` is unavailable.
- Battery is shown from dummy data in dummy mode and hidden gracefully otherwise.
- Plant-health and report data are computed locally until ROS messages arrive.

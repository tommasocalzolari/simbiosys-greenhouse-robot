# UI ROS Connection

The SimBioSys UI currently runs as a ROS 2 Python node, so it connects to ROS through `rclpy` publishers and subscriptions. It does not require rosbridge for the embedded dashboard to run.

The topic config still keeps a `rosbridgeUrl` field for future browser-native ROS integration:

```json
{
  "rosbridgeUrl": "ws://localhost:9090"
}
```

## Start rosbridge

If browser-native ROS access is added later, start rosbridge with:

```bash
source /opt/ros/humble/setup.bash
source /home/mark/MDP/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

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

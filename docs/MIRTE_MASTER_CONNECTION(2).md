# MIRTE Master Connection Notes

Last verified: 2026-04-28

## Robot identity

- IP address on phone hotspot: `10.93.163.146`
- Hostname: `Mirte-6776AC`
- User: `mirte`
- Robot type: MIRTE Master
- SBC/OS seen over SSH: Orange Pi 3B, Armbian, Ubuntu 22.04.5 LTS, ROS 2 Humble
- Current boot launch: `ros2 launch mirte_bringup minimal_master.launch.py`

## Access

- Web UI: `http://10.93.163.146`
- SSH: `ssh mirte@10.93.163.146`
- Onboard VS Code: `http://10.93.163.146/code`
- ROS bridge: `ws://10.93.163.146:9090`

The web UI returned nginx Basic Auth and then served `mirte-web-frontend`.

Open ports observed on the robot included:

- `22`: SSH
- `80`: nginx/web interface
- `3000`, `4567`: MIRTE web backend/frontend services
- `8888`: Python/Jupyter-style web service endpoint
- `9090`: `rosbridge_websocket`

## Active system services

Services observed active on the robot:

- `mirte-ros.service`
- `mirte-web-interface.service`
- `mirte-ap.service`
- `mirte-wifi-watchdog.service`
- `mirte-usb-switch.service`
- `nginx.service`
- `code-server@mirte.service`

The robot also publishes mDNS names:

- `Mirte-6776AC.local`
- `mirte.local`

## Active ROS nodes

Important nodes observed:

- `/io/telemetrix`
- `/controller_manager`
- `/mirte_base_control`
- `/mirte_base_controller`
- `/mirte_master_arm_controller`
- `/mirte_master_gripper_controller`
- `/pid_wheels_controller`
- `/robot_state_publisher`
- `/rplidar_node`
- `/camera/camera`
- `/camera/camera_container`
- `/gripper_camera/gripper_camera`
- `/web_video_server`
- `/rosbridge_websocket`
- `/rosapi`
- `/rosboard_node`

## Active controllers

`ros2 control list_controllers` showed:

- `joint_state_broadcaster`: active
- `mirte_master_arm_controller`: active
- `mirte_master_gripper_controller`: active
- `mirte_base_controller`: active
- `pid_wheels_controller`: active

Hardware components were active for:

- `ros2_gripper_control`
- `ros2_arm_control`
- `mecanumdrive`

## Useful ROS topics

Base:

- `/mirte_base_controller/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/mirte_base_controller/odom` (`nav_msgs/msg/Odometry`)
- `/mirte_base_controller/controller_state`

Arm and gripper:

- `/mirte_master_arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/mirte_master_arm_controller/state`
- `/mirte_master_gripper_controller/transition_event`
- `/joint_states`

Sensors:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/depth/points`
- `/gripper_camera/image_raw`
- `/io/imu/movement/data`
- `/io/distance/rear_left`
- `/io/distance/rear_right`
- `/io/power/power_watcher`
- `/io/encoder/front_left`
- `/io/encoder/front_right`
- `/io/encoder/rear_left`
- `/io/encoder/rear_right`

Low-level motor and servo topics:

- `/io/set_multiple_motor_speeds`
- `/io/motor/front_left/speed`
- `/io/motor/front_right/speed`
- `/io/motor/rear_left/speed`
- `/io/motor/rear_right/speed`
- `/io/servo/hiwonder/shoulder_pan/position`
- `/io/servo/hiwonder/shoulder_lift/position`
- `/io/servo/hiwonder/elbow/position`
- `/io/servo/hiwonder/wrist/position`
- `/io/servo/hiwonder/gripper/position`

## Useful ROS services

Low-level hardware services include:

- `/io/get_board_characteristics`
- `/io/imu/movement/get_data`
- `/io/distance/rear_left/get_range`
- `/io/distance/rear_right/get_range`
- `/io/set_multiple_motor_speeds`
- `/io/motor/*/set_speed`
- `/io/servo/hiwonder/*/set_angle`
- `/io/servo/hiwonder/*/set_angle_with_speed`
- `/io/servo/hiwonder/enable_all_servos`
- `/io/oled/oled/set_text`
- `/io/leds/leds/set_color`
- `/io/power/power_watcher/shutdown`

Camera services include Orbbec controls such as:

- `/camera/get_device_info`
- `/camera/get_sdk_version`
- `/camera/toggle_color`
- `/camera/toggle_depth`
- `/camera/save_images`
- `/camera/save_point_cloud`

ROS bridge/inspection services include many `/rosapi/*` services for listing nodes, topics, services, and message details.

## Packages present on the robot

Relevant ROS packages observed:

- `mirte_bringup`
- `mirte_msgs`
- `mirte_telemetrix_cpp`
- `mirte_master_description`
- `mirte_master_arm_control`
- `mirte_base_control`
- `mirte_moveit_config`
- `mirte_teleop`
- `mirte_zenoh_setup`
- `mirte_fastdds_discovery_setup`
- `rplidar_ros`
- `astra_camera`
- `usb_cam`
- `web_video_server`
- `rosbridge_server`
- `rosapi`
- MoveIt 2 packages
- Some Nav2 packages, including `nav2_map_server`, `nav2_msgs`, `nav2_util`

## Useful commands

Check reachability:

```bash
ping -c 3 10.93.163.146
nc -vz -w 3 10.93.163.146 22
nc -vz -w 3 10.93.163.146 80
curl -I http://10.93.163.146/
```

SSH in:

```bash
ssh mirte@10.93.163.146
```

Inspect robot-side ROS:

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 control list_controllers
ros2 control list_hardware_components
```

Drive the base from the robot shell:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel
```

Move arm up:

```bash
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint'], points: [{positions: [0.0, 0.0, -1.56, 1.56], time_from_start:{ sec: 3, nanosec: 0}}]}"
```

Move arm down:

```bash
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint'], points: [{positions: [0.0, -1.56, -1.56, 1.56], time_from_start:{ sec: 3, nanosec: 0}}]}"
```

Open and close gripper:

```bash
ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -0.1}}"
ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.1}}"
```

## Local development machine context

Local workspace found on this machine:

- `/home/sackmann/mirte_ws`

Local source repos:

- `/home/sackmann/mirte_ws/src/mirte-gazebo`
  - remote: `https://github.com/mirte-robot/mirte-gazebo.git`
  - branch: `main`
  - commit/tag observed: `bb18f29`, tag `0.2.0`
- `/home/sackmann/mirte_ws/src/mirte-ros-packages`
  - remote: `https://github.com/mirte-robot/mirte-ros-packages.git`
  - branch: `develop`
  - commit observed: `2bdd31f`

Local packages in the workspace include:

- `mirte_gazebo`
- `mirte_bringup`
- `mirte_master_description`
- `mirte_master_arm_control`
- `mirte_base_control`
- `mirte_moveit_config`
- `mirte_msgs`
- `mirte_telemetrix_cpp`
- `mirte_teleop`
- `gazebo_grasp_plugin`

Local machine is Ubuntu 22.04.5 with ROS 2 Humble, Gazebo Classic 11, MoveIt 2, and about 395 `ros-humble-*` apt packages installed.

## ROS 2 discovery note

MIRTE docs say ROS 2 communication can be configured in three modes:

- `localhost`: robot-only ROS discovery, MIRTE default
- `zenoh`: remote ROS with Zenoh setup
- `full`: normal ROS 2 DDS discovery on the network

Settings live on the robot in `~/.mirte_settings.sh`. After changing them, restart `mirte-ros.service` or reboot. If local ROS commands do not discover robot topics from the laptop, check this file first.

## Documentation links

- Connect to MIRTE: https://docs.mirte.org/develop/doc/mirte_os/connect_to_mirte.html
- Access web UI and SSH: https://docs.mirte.org/develop/doc/mirte_os/access_interface.html
- MIRTE Master overview: https://docs.mirte.org/develop/doc/robots/mirte_master/index.html
- MIRTE Master control examples: https://docs.mirte.org/develop/doc/simulation/mirte_master_gazebo.html
- ROS 2 communication modes: https://docs.mirte.org/develop/doc/mirte_os/configure/ros2_communication.html
- Programming MIRTE: https://docs.mirte.org/develop/doc/tutorials/programming.html

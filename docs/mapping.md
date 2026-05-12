# Mapping

Run mapping mode:

```bash
ros2 launch simbiosys_bringup mapping_system.launch.py
```

This launches the SimBioSys mission manager, terminal UI if available, keyboard
teleop if available, mapping status helper, and `slam_toolbox` if installed.

Required topic checks:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /mirte_base_controller/odom --once
ros2 run tf2_tools view_frames
```

For the real MIRTE Master, mapping defaults to
`/mirte_base_controller/cmd_vel`, `/mirte_base_controller/odom`, and `/scan`.
For Gazebo or another setup, override them at launch time:

```bash
ros2 launch simbiosys_bringup mapping_system.launch.py \
  cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped \
  odom_topic:=/odom \
  use_sim_time:=true
```

Save a map with the existing Nav2 tool:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/mirte_map
```

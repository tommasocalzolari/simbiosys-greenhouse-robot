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
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

Save a map with the existing Nav2 tool:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/mirte_map
```

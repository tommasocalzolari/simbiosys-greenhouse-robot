# simbiosys_mapping

This package is intentionally a lightweight wrapper/config package.

We reuse `slam_toolbox` for SLAM instead of implementing mapping ourselves. We
reuse `nav2_map_server` and `map_saver_cli` for saving maps.

Before debugging SimBioSys code, verify that MIRTE or Gazebo provides these
topics and transforms:

- `/scan`
- `/mirte_base_controller/odom` on the real MIRTE Master, or `/odom` in some
  simulation setups
- `/tf`
- `/tf_static`

Useful checks:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /mirte_base_controller/odom --once
ros2 run tf2_tools view_frames
```

Save a map with:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/mirte_map
```

# SLAM Debugging and Tuning

Use this checklist when you have the physical MIRTE robot and want to improve
the maps produced by `slam_toolbox`.

The goal is to debug the inputs first, then tune parameters one at a time. Bad
maps are usually caused by one of these:

- ROS discovery or wrong domain.
- Wrong time source.
- Missing or wrong TF transforms.
- Noisy laser scans.
- Wheel odometry drift or jumps.
- Driving too fast for the scan matcher.
- SLAM parameters that do not match the robot or room.

## 1. Prepare the Laptop Terminal

The MIRTE service currently publishes on ROS domain `0`, so use domain `0` on
the laptop unless the robot setup changes.

```bash
cd ~/ro47007_mirte_ws
pixi shell
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
```

Check that the laptop sees robot topics:

```bash
ros2 topic list
```

For debugging stale ROS daemon state, prefer:

```bash
ros2 topic list --no-daemon
```

Expected important topics:

```text
/scan
/tf
/tf_static
/joint_states
/mirte_base_controller/odom
```

## 2. Check the Robot Locally

SSH into the robot and check that the robot service is running:

```bash
sudo systemctl status mirte-ros --no-pager
```

Check the robot's actual ROS domain. On this robot, the service topics appeared
on domain `0`:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic list --no-daemon
```

If the robot itself only shows `/rosout` and `/parameter_events`, fix the robot
bringup first. The laptop cannot map if the robot does not publish topics
locally.

## 3. Run Mapping on the Real Robot

From the laptop:

```bash
cd ~/ro47007_mirte_ws
pixi shell
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start

ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  odom_topic:=/mirte_base_controller/odom \
  output_dir:=maps \
  map_name:=mirte_map
```

`odom_topic` is used by `getmap_node` for monitoring. `slam_toolbox` mostly uses
TF, so the critical transform is still `odom -> base_link`.

Do not run AMCL localization while mapping. Both AMCL and `slam_toolbox` can
publish `map -> odom`, and they should not fight over that transform.

## 4. Check Time Source

For the real robot, `slam_toolbox` must not use simulation time:

```bash
ros2 param get /slam_toolbox use_sim_time
```

Expected:

```text
False
```

If this is `True` during real robot mapping, fix the launch command:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false use_sim_time:=false
```

## 5. Check Laser Scan Data

Check frequency:

```bash
ros2 topic hz /scan
```

Check one message:

```bash
ros2 topic echo /scan --once
```

Look at:

- `header.frame_id`: the lidar frame name.
- `range_min`: minimum valid range.
- `range_max`: maximum valid range.
- `ranges`: should contain many real values, not mostly `inf`, `nan`, or zeros.

Then check that the lidar frame is connected to the robot:

```bash
ros2 run tf2_ros tf2_echo base_link <scan_frame_id>
```

Replace `<scan_frame_id>` with the frame from `/scan`, for example `laser`,
`lidar_link`, or similar.

If this transform fails, the scan cannot be placed correctly on the robot and
the map will be distorted.

## 6. Check Odometry and TF

Check odometry topic frequency:

```bash
ros2 topic hz /mirte_base_controller/odom
```

Check one odometry message:

```bash
ros2 topic echo /mirte_base_controller/odom --once
```

Check the important SLAM transform:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

This must continuously print transforms while the robot is running.

Generate a TF PDF:

```bash
ros2 run tf2_tools view_frames
```

Open the generated PDF and check:

- `map` is published by `slam_toolbox` after mapping starts.
- `map -> odom` exists during mapping.
- `odom -> base_link` exists from the robot base controller.
- `base_link -> <scan_frame_id>` exists.

## 7. Check RViz During Mapping

In RViz:

- Fixed frame should be `map`.
- Add or check displays for `Map`, `LaserScan`, `TF`, and robot model if useful.
- The laser scan should sit on top of real walls/obstacles in the map.
- The map should grow in the correct direction as the robot moves.
- Walls should not split into two parallel copies when driving back over the
  same area.

If RViz says `map` does not exist, wait until `slam_toolbox` receives scans and
TF. If it never appears, check `/scan`, `odom -> base_link`, and time source.

## 8. Driving Procedure for Better Maps

Before changing parameters, drive in a mapping-friendly way:

- Drive slowly.
- Turn slowly.
- Stop briefly after large rotations.
- Avoid fast spinning.
- Avoid pushing the robot by hand unless odometry still updates correctly.
- Keep people and movable objects away from the lidar view.
- Avoid glass, mirrors, thin chair legs, and reflective surfaces when possible.
- Revisit already mapped places so loop closure can correct drift.
- Map one area at a time, then expand.

## 9. Save Test Maps with Different Names

Use a different map name for each experiment:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  odom_topic:=/mirte_base_controller/odom \
  map_name:=test_01_baseline
```

Maps are saved in:

```text
maps/test_01_baseline.yaml
maps/test_01_baseline.pgm
```

Keep notes for each test:

| Test | Changed parameter | Route | Result |
| --- | --- | --- | --- |
| `test_01_baseline` | none | slow loop around room | walls slightly doubled |
| `test_02_range_5m` | `max_laser_range: 5.0` | same route | less far noise |

## 10. Record a ROS Bag for Repeatable Tuning

Record the important data once:

```bash
mkdir -p bags
ros2 bag record -o bags/slam_debug_01 \
  /scan \
  /tf \
  /tf_static \
  /mirte_base_controller/odom
```

Replay it later to compare parameter changes without driving again:

```bash
ros2 bag play bags/slam_debug_01 --clock
```

When replaying a bag with `--clock`, run mapping with simulation time:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  use_sim_time:=true \
  odom_topic:=/mirte_base_controller/odom \
  map_name:=bag_test_01
```

Do not drive the real robot while using bag replay.

## 11. Tuning Rules

Tune one parameter at a time.

For each tuning run:

1. Save the old parameter value.
2. Change one parameter.
3. Use the same route and similar speed.
4. Save the map with a new name.
5. Compare wall sharpness, duplicated walls, drift, and loop closure behavior.
6. Keep the change only if the map is clearly better.

After changing `src/simbiosys_mapping/config/slam_toolbox_mapping.yaml`, rebuild
or use symlink install behavior if available, then source again:

```bash
colcon build --packages-select simbiosys_mapping
source install/setup.bash
```

## 12. Recommended Parameter Order

Start with the parameters in
`src/simbiosys_mapping/config/slam_toolbox_mapping.yaml`.

### `max_laser_range`

Controls how far laser readings are used.

If the map has noisy far walls, ghost obstacles, or distorted open spaces, reduce
this first.

Try:

```yaml
max_laser_range: 5.0
```

Then try:

```yaml
max_laser_range: 4.0
```

If the room is large and scans are clean, increase carefully.

### `minimum_time_interval`

Minimum time between scans used by SLAM.

Current useful range:

```yaml
minimum_time_interval: 0.1
minimum_time_interval: 0.2
minimum_time_interval: 0.5
```

Lower values use more scans and can improve detail, but cost more CPU and may
add noise if scans are very similar.

### `minimum_travel_distance`

Minimum movement before adding a new scan to the SLAM graph.

Try:

```yaml
minimum_travel_distance: 0.1
minimum_travel_distance: 0.2
minimum_travel_distance: 0.3
```

Lower values add more scans. Higher values reduce CPU and repeated noisy scans.

### `minimum_travel_heading`

Minimum rotation before adding a new scan.

Try:

```yaml
minimum_travel_heading: 0.1
minimum_travel_heading: 0.2
minimum_travel_heading: 0.3
```

`0.2` rad is about 11.5 degrees.

If maps get bad during turns, reduce this and drive slower.

### `resolution`

Map cell size in meters.

Common values:

```yaml
resolution: 0.05
resolution: 0.03
```

`0.05` is a good default. `0.03` gives more detail but costs more CPU/memory.

### `map_update_interval`

How often `/map` updates.

This mostly affects how quickly RViz and auto-save see changes, not the core map
quality.

Try:

```yaml
map_update_interval: 2.0
map_update_interval: 5.0
```

### `do_loop_closing`

Loop closure corrects accumulated drift when returning to a known place.

Usually keep:

```yaml
do_loop_closing: true
```

If loop closure causes sudden bad jumps, keep it true but first debug odometry,
scan quality, and driving path. Turning it off is only a temporary diagnostic.

### `transform_timeout`

How long SLAM waits for TF.

If logs show TF timeout warnings, try:

```yaml
transform_timeout: 0.5
```

Do not hide a broken TF tree with a large timeout. First check `tf2_echo`.

## 13. Symptom Guide

| Symptom | Likely cause | What to check or change |
| --- | --- | --- |
| Only `/rosout` and `/parameter_events` visible | Wrong domain or daemon state | Use `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`, restart daemon |
| `map` frame never appears | No scans, bad TF, wrong time | Check `/scan`, `odom -> base_link`, `use_sim_time` |
| Walls are doubled | Odometry drift, fast driving, weak loop closure | Drive slower, revisit areas, check odom, lower `minimum_time_interval` |
| Walls are fuzzy/thick | Too many noisy scans or bad lidar range | Reduce `max_laser_range`, check lidar frame |
| Map bends like a curve | Odometry/TF error | Check `odom -> base_link`, wheel slip, timestamps |
| Map jumps suddenly | Loop closure accepted a bad match | Check scan quality, driving path, loop closure behavior |
| RViz map updates slowly | Map publish interval | Lower `map_update_interval` |
| CPU too high | Too many scans or high resolution | Increase `minimum_time_interval`, increase travel thresholds, keep `resolution: 0.05` |

## 14. Final Quality Check

A map is good enough when:

- Walls look straight where the real walls are straight.
- Returning to the start does not create duplicated walls.
- The map is not rotated or scaled incorrectly.
- Obstacles appear in the correct relative positions.
- The saved `.yaml` and `.pgm` files open correctly.
- AMCL localization can start on the saved map and keep the robot pose stable.

After a good map is saved, test localization:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

Then use RViz `2D Pose Estimate` to initialize AMCL.

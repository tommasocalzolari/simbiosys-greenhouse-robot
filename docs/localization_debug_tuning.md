# Localization Debugging and Tuning

Use this checklist when testing AMCL localization on the physical MIRTE robot.
Localization uses a saved map; it should not build or change the map.

## 1. Start From the Correct ROS Environment

On the laptop:

```bash
cd ~/ro47007_mirte_ws
pixi shell
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
```

Check robot topics:

```bash
ros2 topic list
```

Required topics:

```text
/scan
/tf
/tf_static
/robot_description
/mirte_base_controller/odom
```

## 2. Launch Localization

Do not run `slam_toolbox` at the same time.

```bash
ros2 node list | grep slam
```

Start AMCL:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

In RViz, use `2D Pose Estimate` to set the robot pose accurately.

## 3. Verify Nodes and Topics

Check lifecycle states:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

Both should be active.

Check map and AMCL outputs:

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /particle_cloud --once
```

After the initial pose is set, check:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 4. Verify Laser and TF Alignment

Find the laser frame:

```bash
ros2 topic echo /scan --once
```

Use the value of `header.frame_id`:

```bash
ros2 run tf2_ros tf2_echo base_link <scan_frame_id>
```

In RViz:

- Fixed frame: `map`
- Enable `Map`, `LaserScan`, `ParticleCloud`, and `TF`
- After setting initial pose, the laser scan should overlap the map walls

If the scan is offset or rotated even while the robot is still, suspect a wrong
initial pose or wrong lidar TF.

## 5. Basic Test Sequence

Run these tests in order. Save notes after each one.

### Static Test

1. Set initial pose.
2. Do not move for 20 seconds.
3. Watch `/amcl_pose` and particles.

Expected: pose stays stable and particles cluster around the robot.

### Slow Straight Test

1. Drive forward slowly 1-2 meters.
2. Stop.
3. Check scan/map alignment.

If pose drifts while driving straight, check odometry scale and map quality.

### Slow Turn Test

1. Rotate 90 degrees slowly.
2. Stop.
3. Check scan/map alignment.
4. Repeat for another 90 degrees.

If pose breaks during turns, odometry yaw or AMCL motion noise is the likely
problem.

### Sharp Turn Test

Repeat the turn test with sharper turns.

If only sharp turns fail, tune AMCL to trust odometry less and update more often.

### Map Edge Test

Drive near the edge of the saved map.

If localization weakens only there, the map may be too small or the robot sees
too much unknown space.

## 6. Tuning Order

Tune one change at a time and repeat the same test route.

### Motion Noise: `alpha1` to `alpha5`

Increase these if AMCL trusts bad odometry too much.

- `alpha1`: rotation noise from rotation.
- `alpha2`: rotation noise from translation.
- `alpha3`: translation noise from translation.
- `alpha4`: translation noise from rotation.
- `alpha5`: lateral/sideways noise for omni bases.

For turn drift, try:

```yaml
alpha1: 0.5
alpha2: 0.5
alpha5: 0.5
```

If localization becomes too jumpy, reduce them again.

### Update Frequency: `update_min_a`, `update_min_d`

Lower values make AMCL update more often.

For small robots, useful values are:

```yaml
update_min_a: 0.1
update_min_d: 0.1
```

If CPU is high, increase them slightly.

### Particle Count: `min_particles`, `max_particles`

More particles improve robustness but cost CPU.

Try:

```yaml
min_particles: 1000
max_particles: 3000
```

If AMCL loses pose in ambiguous areas, increase `max_particles`.

### Laser Range: `laser_max_range`

Reduce this if far readings are noisy or do not match the map:

```yaml
laser_max_range: 5.0
```

Use a value that matches the reliable part of the lidar and the mapped area.

### Laser Matching: `laser_likelihood_max_dist`

Controls how far from an obstacle a laser endpoint can still help matching.

Try small changes:

```yaml
laser_likelihood_max_dist: 1.0
laser_likelihood_max_dist: 2.0
```

## 7. Symptom Guide

| Symptom | Likely cause | First check |
| --- | --- | --- |
| Map not visible | map server inactive or bad map path | `/map_server`, `/map` |
| No `map -> odom` | initial pose missing or AMCL inactive | RViz pose estimate, `/amcl` |
| Good at first, bad after turns | odometry yaw error or low motion noise | `alpha1`, `alpha2`, `alpha5` |
| Bad near map edges | too much unknown space | make a larger map |
| Laser never matches map | wrong initial pose or lidar TF | `base_link -> scan_frame` |
| Particles spread everywhere | weak scan match or bad initial pose | reset initial pose, check map |
| Pose jumps suddenly | ambiguous map or too much noise | map quality, particles, laser range |

## 8. Record a Bag for Repeatable Tuning

Record:

```bash
mkdir -p bags
ros2 bag record -o bags/localization_debug_01 \
  /scan \
  /tf \
  /tf_static \
  /mirte_base_controller/odom \
  /amcl_pose \
  /particle_cloud
```

Replay with clock:

```bash
ros2 bag play bags/localization_debug_01 --clock
```

When replaying a bag, launch localization with:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  use_sim_time:=true \
  map:=maps/mirte_map.yaml
```

## 9. After Changing Parameters

After editing `src/simbiosys_mapping/config/amcl_localization.yaml`:

```bash
colcon build --packages-select simbiosys_mapping
source install/setup.bash
```

Then relaunch localization and repeat the same test route.

# Strafing Alignment Test

This document explains the lidar-based strafing code currently implemented in
`simbiosys_perception`.

The goal is to let the robot drive sideways parallel to a surface while keeping
a target distance from it. The current implementation is a test stack, not yet a
full mission behavior.

## Nodes

There are two nodes involved:

- `bed_side_alignment_node`
- `alignment_strafe_test_node`

The alignment node reads lidar data and publishes an alignment estimate. The
strafe test node reads that estimate and publishes base velocity commands.

## Topics

Input topics:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/simbiosys/bed_side_alignment`
  (`simbiosys_interfaces/msg/BedSideAlignment`)

Output topic:

- `/mirte_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`)

The alignment node publishes:

```text
/simbiosys/bed_side_alignment
```

The strafe node publishes:

```text
/mirte_base_controller/cmd_vel_unstamped
```

The strafe node also subscribes to `/scan` directly to prevent sideways motion
into a close side wall or inner corner.

Velocity command publishing uses queue depth `1` by default, so old movement
commands are dropped instead of being buffered.

## Alignment Node

Run:

```bash
source install/setup.bash
ros2 run simbiosys_perception bed_side_alignment_node
```

Check output:

```bash
ros2 topic echo /simbiosys/bed_side_alignment
```

The alignment node uses the lidar to fit one straight line to a surface. By
default it looks in front of the robot:

```text
-45 degrees to +45 degrees
```

It filters lidar points by range:

```text
0.15m to 4.0m
```

The default `scan_angle_offset_deg` is `90.0`, matching the current real robot
lidar orientation. If you run in simulation, override it to `0.0` on both nodes.

Then it publishes:

- `valid`: whether a usable line was found.
- `distance_m`: distance from lidar to the fitted surface.
- `target_distance_m`: desired distance to keep.
- `distance_error_m`: `distance_m - target_distance_m`.
- `yaw_error_rad`: angle error relative to the fitted surface.
- `confidence`: confidence based on fit quality.
- `message`: debug text with point count and fit info.

Default target distance:

```text
0.35m
```

Override it:

```bash
ros2 run simbiosys_perception bed_side_alignment_node --ros-args \
  -p target_distance_m:=0.45
```

## Strafe Test Node

Run with motion enabled by default:

```bash
source install/setup.bash
ros2 run simbiosys_perception alignment_strafe_test_node
```

Run without motion:

```bash
source install/setup.bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p enable_motion:=false
```

Common simulation command:

```bash
source install/setup.bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped \
  -p scan_angle_offset_deg:=0.0
```

Real robot command:

```bash
source install/setup.bash
ros2 run simbiosys_perception alignment_strafe_test_node
```

## Sequential Strafe Test Node

There is also a stricter copied test node:

```bash
source install/setup.bash
ros2 run simbiosys_perception sequential_alignment_strafe_test_node
```

It uses the same parameters and topics as `alignment_strafe_test_node`, but the
motion logic is mutually exclusive:

- outside the strafe gates, it only aligns with `linear.x` and `angular.z`
- inside the strafe gates, it only strafes with `linear.y`

This means it does not strafe while aligning, and it does not keep correcting
distance/yaw once it starts strafing.

Real robot example:

```bash
source install/setup.bash
ros2 run simbiosys_perception sequential_alignment_strafe_test_node
```

Watch commands:

```bash
ros2 topic echo /mirte_base_controller/cmd_vel
```

Stop other velocity publishers, such as `teleop_twist_keyboard`, before testing.
Otherwise multiple nodes can publish to the same command topic and fight each
other.

## Behavior

The strafe node prioritizes alignment before sideways motion.

The regular `alignment_strafe_test_node` can keep correcting distance and yaw
while it strafes. The `sequential_alignment_strafe_test_node` does not mix those
motions.

It only allows `linear.y` strafing when:

- the alignment message is valid
- confidence is high enough
- distance error is within `strafe_distance_tolerance_m`
- yaw error is within `strafe_yaw_tolerance_rad`
- side clearance in the strafe direction is safe

Even while strafing, it still corrects:

- distance with `linear.x`
- yaw with `angular.z`

Default strafe gates:

```text
strafe_distance_tolerance_m: 0.10
strafe_yaw_tolerance_rad: 5 degrees
```

Default correction deadbands:

```text
distance_tolerance_m: 0.01
yaw_tolerance_rad: 1 degree
```

Default side clearance:

```text
min_side_clearance_m: 1.00
side_clearance_ignore_below_m: 0.02
invert_side_clearance_side: false
left strafe sector: 70 to 110 degrees
right strafe sector: -110 to -70 degrees
```

If the robot is strafing left and the left lidar sector sees something closer
than `min_side_clearance_m`, the node stops sideways movement. It still keeps alignment
corrections active.

Default queue depths:

```text
cmd_vel_queue_depth: 1
input_queue_depth: 1
```

## Useful Tuning

Strafe speed:

```bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p strafe_speed_mps:=0.20
```

Yaw correction:

```bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p yaw_gain:=1.4
```

Maximum angular speed:

```bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p max_angular_speed_radps:=1.2
```

Side clearance:

```bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p min_side_clearance_m:=0.40
```

If the robot is strafing left but the clearance guard appears to watch the right
side, flip the clearance sector:

```bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p invert_side_clearance_side:=true
```

Scan angle offset:

```bash
ros2 run simbiosys_perception bed_side_alignment_node --ros-args \
  -p scan_angle_offset_deg:=0.0
```

Strafe direction:

```bash
-p strafe_direction:=left
-p strafe_direction:=right
```

## Debugging

Check scan:

```bash
ros2 topic echo /scan --once
```

Check alignment:

```bash
ros2 topic echo /simbiosys/bed_side_alignment
```

Check command publishers:

```bash
ros2 topic info -v /mirte_base_controller/cmd_vel_unstamped
```

Check all related topics:

```bash
ros2 topic list -t | grep -E 'scan|alignment|cmd_vel'
```

If alignment says `not enough points`, the lidar may not see enough usable
points in the selected region or range window.

If the robot does not strafe, check:

- `enable_motion` has not been set to `false`.
- `/simbiosys/bed_side_alignment` has `valid: true`.
- confidence is above `min_confidence`.
- distance/yaw are within the strafe gates.
- side clearance is not blocking motion.
- no other node is publishing conflicting velocity commands.

## Build After Changes

```bash
colcon build --packages-select simbiosys_perception
source install/setup.bash
```

Run the geometry tests:

```bash
PYTHONPATH=src/simbiosys_perception pytest src/simbiosys_perception/test/test_surface_fit.py
```

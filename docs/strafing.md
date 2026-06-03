# Strafing And Bed-Side Control

This document covers the two lidar-based strafing paths that currently coexist
in SimBioSys:

- bin strafing: a direct action that strafes along a detected bin wall until the
  wall endpoint/corner is reached
- bed-side scanning: the mission-facing bed-side controller that aligns,
  strafes, reports scan progress, and counts flower detections

Both paths publish `geometry_msgs/msg/Twist` commands. The real robot default is
`/mirte_base_controller/cmd_vel`; use `/mirte_base_controller/cmd_vel_unstamped`
in simulation when the launch file or controller expects it.

## Bin Strafe Action

The bin strafe stack is the direct low-level strafe action. It assumes the robot
faces the flower bin wall, then:

- strafes left or right with `linear.y`
- corrects bin distance with `linear.x`
- corrects wall angle with `angular.z`
- stops when the fitted wall endpoint reaches the active strafe direction

Alignment topic:

```text
/simbiosys/bin_wall_alignment
simbiosys_interfaces/msg/BinWallAlignment
```

Strafe action:

```text
/simbiosys/execute_bin_strafe
simbiosys_interfaces/action/ExecuteBinStrafe
```

Start the alignment node:

```bash
source install/setup.bash
ros2 run simbiosys_perception bin_wall_alignment_node
```

Watch the estimate:

```bash
ros2 topic echo /simbiosys/bin_wall_alignment
```

Important fields:

- `valid`: a straight bin wall segment was found
- `corner_detected`: the endpoint in the strafe direction is close enough
- `distance_error_m`: `distance_m - target_distance_m`
- `yaw_error_rad`: angle error from the expected wall angle
- `wall_start_m` / `wall_end_m`: wall extents along the strafe axis
- `endpoint_in_direction_m`: remaining wall extent in the active strafe
  direction

Start the action server:

```bash
ros2 run simbiosys_behavior bin_strafe_action_node --ros-args \
  -p enable_motion:=true
```

Run a strafe:

```bash
ros2 action send_goal /simbiosys/execute_bin_strafe \
  simbiosys_interfaces/action/ExecuteBinStrafe \
  "{direction: left}"
```

Useful tuning:

```bash
ros2 run simbiosys_perception bin_wall_alignment_node --ros-args \
  -p target_distance_m:=0.35 \
  -p scan_angle_offset_rad:=-1.5708 \
  -p roi_min_angle_rad:=-1.05 \
  -p roi_max_angle_rad:=1.05 \
  -p cluster_jump_m:=0.05 \
  -p max_fit_error_m:=0.025 \
  -p min_wall_length_m:=0.30 \
  -p corner_endpoint_threshold_m:=0.10

ros2 run simbiosys_behavior bin_strafe_action_node --ros-args \
  -p enable_motion:=true \
  -p default_strafe_speed_mps:=0.08 \
  -p max_strafe_speed_mps:=0.08 \
  -p strafe_distance_tolerance_m:=0.04 \
  -p strafe_yaw_tolerance_rad:=0.087 \
  -p min_confidence:=0.45 \
  -p corner_confirmations:=3
```

## Bed-Side Scan Controller

The bed-side controller is the mission-facing implementation for
`ExecuteBedSideScan`. It subscribes to `BedSideAlignment`, `FlowerTarget`, and
`LaserScan`; it publishes velocity commands and scan progress while the action
is active.

Alignment topic:

```text
/simbiosys/bed_side_alignment
simbiosys_interfaces/msg/BedSideAlignment
```

Scan action:

```text
/simbiosys/execute_bed_side_scan
simbiosys_interfaces/action/ExecuteBedSideScan
```

Progress topic:

```text
/simbiosys/scan_progress
simbiosys_interfaces/msg/ScanProgress
```

Start the alignment node:

```bash
source install/setup.bash
ros2 run simbiosys_perception bed_side_alignment_node --ros-args \
  -p target_distance_m:=0.35
```

Start the controller without motion for dry testing:

```bash
ros2 run simbiosys_behavior bed_side_controller_node --ros-args \
  -p enable_motion:=false
```

Enable motion only in a safe test area:

```bash
ros2 run simbiosys_behavior bed_side_controller_node --ros-args \
  -p enable_motion:=true \
  -p cmd_vel_topic:=/mirte_base_controller/cmd_vel
```

The controller only strafes when:

- the alignment message is fresh and valid
- confidence is above `min_confidence`
- distance error is within `strafe_distance_tolerance_m`
- yaw error is within `strafe_yaw_tolerance_rad`
- side clearance in the strafe direction is safe

While strafing, it can still correct:

- distance with `linear.x`
- yaw with `angular.z`

It reports success when an end-of-bed condition is confirmed. End detection can
come from a front scan gap, optional side blockage, or alignment loss after
strafing has started.

Default real-robot parameters:

```text
cmd_vel_topic: /mirte_base_controller/cmd_vel
cmd_vel_queue_depth: 1
input_queue_depth: 1
scan_angle_offset_deg: 90.0
strafe_distance_tolerance_m: 0.10
strafe_yaw_tolerance_rad: 5 deg
min_side_clearance_m: 0.50
side_clearance_ignore_below_m: 0.02
left strafe sector: 75 to 105 deg
right strafe sector: -105 to -75 deg
```

## Strafe Test Nodes

`alignment_strafe_test_node` is a small guarded test node for validating lateral
base motion from `BedSideAlignment` and `LaserScan`. It publishes one current
command at a time with queue depth `1`.

Run on the real robot:

```bash
source install/setup.bash
ros2 run simbiosys_perception alignment_strafe_test_node
```

Run in simulation:

```bash
source install/setup.bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped \
  -p scan_angle_offset_deg:=0.0
```

Run without motion:

```bash
ros2 run simbiosys_perception alignment_strafe_test_node --ros-args \
  -p enable_motion:=false
```

`sequential_alignment_strafe_test_node` uses the same parameters and topics but
does not mix alignment corrections with sideways motion:

- outside the strafe gates, it only aligns with `linear.x` and `angular.z`
- inside the strafe gates, it only strafes with `linear.y`

## Debugging

Check scan:

```bash
ros2 topic echo /scan --once
```

Check bed-side alignment:

```bash
ros2 topic echo /simbiosys/bed_side_alignment
```

Check command publishers:

```bash
ros2 topic info -v /mirte_base_controller/cmd_vel
```

Check all related topics:

```bash
ros2 topic list -t | grep -E 'scan|alignment|cmd_vel|bed_side|bin_wall'
```

If the robot does not strafe, check:

- `enable_motion` has not been set to `false`
- no other node is publishing conflicting velocity commands
- `/simbiosys/bed_side_alignment` or `/simbiosys/bin_wall_alignment` is valid
- confidence is above `min_confidence`
- distance/yaw are within the strafe gates
- side clearance is not blocking motion

## Build And Test

```bash
colcon build --packages-select simbiosys_interfaces simbiosys_perception simbiosys_behavior
source install/setup.bash
PYTHONPATH=src/simbiosys_perception pytest \
  src/simbiosys_perception/test/test_surface_fit.py \
  src/simbiosys_perception/test/test_bin_wall_fit.py
```

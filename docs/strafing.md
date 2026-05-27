# Bin Strafing

This is the fresh lidar-based strafing stack. It is separate from the older
bed-side alignment test nodes.

The robot is assumed to face the flower bin wall. The controller then:

- strafes left/right with `linear.y`
- corrects bin distance with `linear.x`
- corrects wall angle with `angular.z`
- stops successfully when the fitted wall endpoint reaches the robot's strafe
  direction, which means the bin corner has been reached

The MIRTE lidar is mounted with a 90 degree scan offset in the current setup.
The alignment node subtracts `scan_angle_offset_rad` before filtering and
fitting points. The default is `-1.5708` rad.

## Interfaces

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

Velocity output:

```text
/mirte_base_controller/cmd_vel
```

Use `/mirte_base_controller/cmd_vel_unstamped` in simulation if needed.

## Start Alignment

The alignment node defaults are set for the current robot:

```text
scan_angle_offset_rad: -1.5708
strafe_direction: left
target_distance_m: 0.35
roi: -60 deg to +60 deg in robot-front coordinates
```

Run the alignment node (example shown as used locally):

```bash
ros2 run simbiosys_perception bin_wall_alignment_node
```

Watch the estimate:

```bash
ros2 topic echo /simbiosys/bin_wall_alignment
```

Important fields:

- `valid`: a straight bin wall segment was found.
- `corner_detected`: the endpoint in the strafe direction is close enough.
- `distance_error_m`: `distance_m - target_distance_m`.
- `yaw_error_rad`: angle error from the expected wall angle.
- `wall_start_m` / `wall_end_m`: wall extents along the strafe axis.
- `endpoint_in_direction_m`: remaining wall extent in the active strafe
  direction.

## Start Strafe Action Server

Run the action server. In local testing we usually start it with `enable_motion` enabled:

```bash
ros2 run simbiosys_behavior bin_strafe_action_node --ros-args \
  -p enable_motion:=true
```

Note: `enable_motion:=true` allows the node to publish real velocity commands; leave it `false` for dry testing.


## Run A Strafe

Simple run (example used locally):

```bash
ros2 action send_goal /simbiosys/execute_bin_strafe \
  simbiosys_interfaces/action/ExecuteBinStrafe \
  "{direction: left}"
```

You can pass additional fields (for example `dry_run`, `target_distance_m`, `strafe_speed_mps`, and `timeout_sec`) when tuning or performing dry tests.

When the bin corner is confirmed for several scans, the action stops motion and
returns:

```text
success: true
message: succeeded strafing: corner reached
```

## Tuning

Optional alignment overrides:

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
```

Controller parameters:

```bash
ros2 run simbiosys_behavior bin_strafe_action_node --ros-args \
  -p enable_motion:=true \
  -p default_strafe_speed_mps:=0.08 \
  -p max_strafe_speed_mps:=0.08 \
  -p strafe_distance_tolerance_m:=0.04 \
  -p strafe_yaw_tolerance_rad:=0.087 \
  -p min_confidence:=0.45 \
  -p corner_confirmations:=3
```

## Build And Test

```bash
colcon build --packages-select simbiosys_interfaces simbiosys_perception simbiosys_behavior
source install/setup.bash
PYTHONPATH=src/simbiosys_perception pytest \
  src/simbiosys_perception/test/test_bin_wall_fit.py
```

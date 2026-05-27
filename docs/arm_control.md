# Arm Control

SimBioSys wraps MIRTE arm and gripper interfaces instead of replacing them.

Known MIRTE interfaces:

```text
/mirte_master_arm_controller/joint_trajectory
/mirte_master_arm_controller/follow_joint_trajectory
/mirte_master_gripper_controller/gripper_cmd
/io/servo/hiwonder/gripper/get_range
/enable_arm_control
/joint_states
```

Start safe wrapper nodes:

```bash
ros2 launch simbiosys_bringup arm_test.launch.py
```

Slow the arm motion by increasing the named pose duration, for example:

```bash
ros2 launch simbiosys_bringup arm_test.launch.py motion_duration_sec:=5.0
```

Run this from a Pixi shell after a successful build:

```bash
cd /path/to/main-simbiosys
pixi shell
source install/setup.bash
export ROS_LOCALHOST_ONLY=0
ros2 launch simbiosys_bringup arm_test.launch.py
```

Use a second Pixi shell for service calls:

```bash
cd /path/to/main-simbiosys
pixi shell
source install/setup.bash
export ROS_LOCALHOST_ONLY=0
```

Monitor joints:

```bash
ros2 topic echo /joint_states --once
```

Send a safe placeholder named pose:

```bash
ros2 service call /simbiosys/send_named_arm_pose simbiosys_interfaces/srv/SendNamedArmPose "{pose_name: home}"
```

Open or close the placeholder gripper client:

```bash
ros2 service call /simbiosys/set_gripper_closed std_srvs/srv/SetBool "{data: false}"
ros2 service call /simbiosys/set_gripper_closed std_srvs/srv/SetBool "{data: true}"
```

The current MIRTE Master reports the gripper servo range as approximately
`-0.7603` to `0.6458` radians:

```bash
ros2 service call /io/servo/hiwonder/gripper/get_range mirte_msgs/srv/GetServoRange "{}"
```

The wrapper keeps conservative placeholder positions by default and clamps
requests to that observed range. Override the placeholders during supervised
testing if the physical open/closed directions are calibrated:

```bash
ros2 launch simbiosys_bringup arm_test.launch.py \
  gripper_open_position:=0.04 \
  gripper_close_position:=0.0
```

Keep all real hardware motion slow, supervised, and tested in simulation first.

## Flower Pick Routine

`simbiosys_arm` also contains a simple callable flower picking routine:

```text
ros2 run simbiosys_arm flower_pick_node
```

The node is meant as a demo-friendly arm routine, not a full autonomous arm
planner. It starts from `stow`, opens the gripper, moves up and forward toward
the flower, closes below the flower head, lifts, drops at a fixed storage pose
90 degrees left of the robot, opens the gripper, and returns to `stow`.

Default sequence:

```text
move_stow_start
open_gripper
move_ready_above
move_inspect
move_pre_grasp
move_grasp
close_gripper
lift
move_storage
open_gripper_drop
move_stow_end
```I implemeted some bedside allignment streafe mode. however this is not working at all so i would like to start from scratch. somehow i need to get a strafe mode where the robot strafes left or right next to a flower bin. i want it to get allignment based on either lidar or depth cam. prefer lidar due to information size to get from the robot. the allignment should work on straight walls with small legs sticking out of about 1x1 cm, and then like 50 cm of straight wall. how would you tackle this problem. it should also handle corners . so at a corner it should not try to go around the corner

The default motion backend is `simple_ik`. This keeps the demo independent from
RViz/MoveIt planning for the actual run. The simple IK:

- clamps arm joints to `[-pi/2, pi/2]`
- keeps the wrist mostly horizontal with
  `shoulder_lift + elbow + wrist = -pi/2`
- uses the flower target height as the main grasp height
- uses bbox `x` for small shoulder pan correction
- uses bbox `y` for small height correction
- uses a fixed/tunable forward distance because the current `FlowerTarget`
  message does not carry depth

### Start The Node

Use a Pixi shell and source the workspace:

```bash
cd /path/to/main-simbiosys
pixi shell
source install/setup.bash
export ROS_LOCALHOST_ONLY=0
ros2 run simbiosys_arm flower_pick_node
```

Useful optional parameters:

```bash
ros2 run simbiosys_arm flower_pick_node --ros-args \
  -p flower_distance_m:=0.28 \
  -p fallback_flower_height_m:=0.20 \
  -p grasp_below_head_m:=0.015 \
  -p lift_above_grasp_m:=0.08 \
  -p joint_motion_duration_sec:=2.5
```

### Call The Pick Service

The picker exposes a manual trigger service:

```text
/simbiosys/pick_flower
std_srvs/srv/Trigger
```

Call it from a second sourced Pixi shell:

```bash
ros2 service call /simbiosys/pick_flower std_srvs/srv/Trigger "{}"
```

The response contains:

```text
success: true/false
message: human-readable step or error text
```

Example failure message:

```text
Pick failed during move_grasp: simple_ik could not reach grasp; closest error=0.052m
```

### Flower Target Input

The picker subscribes to:

```text
/simbiosys/flower_target
simbiosys_interfaces/msg/FlowerTarget
```

Important fields:

```text
detected              must be true
ready_for_harvest    checked only if require_ready_for_harvest is true
bbox_center_px.x     image-space left/right correction for shoulder pan
bbox_center_px.y     image-space up/down correction for grasp height
height_cm            flower head height from the ground
confidence           checked against min_confidence
flower_id            used in logs
```

If no target has been received, the node uses a fake centered target by default
so the routine can still be tested in simulation:

```text
use_fake_target_if_missing:=true
fake_bbox_center_x_px:=320.0
fake_bbox_center_y_px:=240.0
fallback_flower_height_m:=0.20
```

Publish a manual test target:

```bash
ros2 topic pub --once /simbiosys/flower_target simbiosys_interfaces/msg/FlowerTarget \
  "{flower_id: 'test_flower',
    detected: true,
    ready_for_harvest: true,
    bbox_center_px: {x: 320.0, y: 240.0, z: 0.0},
    height_cm: 20.0,
    confidence: 1.0,
    message: 'manual test target'}"
```

Then call:

```bash
ros2 service call /simbiosys/pick_flower std_srvs/srv/Trigger "{}"
```

### Commands Sent By The Picker

The picker sends arm motion to:

```text
/mirte_master_arm_controller/follow_joint_trajectory
control_msgs/action/FollowJointTrajectory
```

It sends gripper commands to:

```text
/mirte_master_gripper_controller/gripper_cmd
control_msgs/action/GripperCommand
```

It optionally calls the arm enable service if available:

```text
/enable_arm_control
std_srvs/srv/SetBool
```

The node logs each step as it runs. Watch the terminal where
`flower_pick_node` is running for the quickest debug feedback.

### Important Tuning Parameters

Forward distance and height:

```text
flower_distance_m           fixed forward distance to the flower head
fallback_flower_height_m    used when FlowerTarget.height_cm is missing
grasp_below_head_m          how far below the flower head to close fingers
lift_above_grasp_m          lift after closing
ready_distance_m            first high/near approach position
ready_above_head_m          height above flower before extending forward
```

Bbox correction:

```text
image_width_px
image_height_px
bbox_x_to_pan_gain_rad_per_px
max_bbox_pan_offset_rad
bbox_y_to_z_gain_m_per_px
max_bbox_z_offset_m
```

If the arm pans the wrong way when the bbox is left/right of center, flip the
sign of `bbox_x_to_pan_gain_rad_per_px`.

If the arm moves the grasp height the wrong way when the bbox is high/low, flip
the sign of `bbox_y_to_z_gain_m_per_px`.

### Simulation Notes

The currently used MIRTE Master Gazebo launch publishes the body depth camera:

```text
/camera/image_raw
/camera/depth/image_raw
/camera/points
```

That camera is mounted to `frame_link`, not to the gripper. No wrist/gripper
camera is currently present in the default simulation launch.

For base teleop in simulation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

For RViz from the Pixi environment, make sure RViz uses the Pixi OGRE plugins:

```bash
PIXI_PREFIX="$PWD/.pixi/envs/default"
export OGRE_RESOURCE_PATH="$PIXI_PREFIX/opt/rviz_ogre_vendor/lib/OGRE"
export LD_LIBRARY_PATH="$PWD/install/lib:$PIXI_PREFIX/opt/rviz_ogre_vendor/lib:$PIXI_PREFIX/lib:$PIXI_PREFIX/lib/gazebo-11/plugins:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
rviz2
```

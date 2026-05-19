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

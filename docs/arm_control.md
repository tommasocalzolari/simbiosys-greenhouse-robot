# Arm Control

SimBioSys wraps MIRTE arm and gripper interfaces instead of replacing them.

Known MIRTE interfaces:

```text
/mirte_master_arm_controller/joint_trajectory
/mirte_master_arm_controller/follow_joint_trajectory
/mirte_master_gripper_controller/gripper_cmd
/joint_states
```

Start safe wrapper nodes:

```bash
ros2 launch simbiosys_bringup arm_test.launch.py
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

Keep all real hardware motion slow, supervised, and tested in simulation first.

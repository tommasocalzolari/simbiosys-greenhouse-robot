# Topic Reference

The default topic names are also captured in:

```text
src/simbiosys_bringup/config/real_robot_topics.yaml
src/simbiosys_bringup/config/simulation_topics.yaml
```

| Purpose | Topic or Action |
| --- | --- |
| Base velocity | `/mirte_base_controller/cmd_vel_unstamped` |
| Joint states | `/joint_states` |
| Arm trajectory | `/mirte_master_arm_controller/joint_trajectory` |
| Arm FollowJointTrajectory action | `/mirte_master_arm_controller/follow_joint_trajectory` |
| Gripper action | `/mirte_master_gripper_controller/gripper_cmd` |
| Laser scan | `/scan` |
| Odometry | `/odom` |
| Map | `/map` |
| Color camera image | `/camera/color/image_raw` |
| Depth camera image | `/camera/depth/image_raw` |
| Depth point cloud | `/camera/depth/points` |
| Gripper camera image | `/gripper_camera/image_raw` |
| Task status | `simbiosys/task_status` |
| Flower data | `simbiosys/flower_data` |

Verification checklist:

```bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 action list
```

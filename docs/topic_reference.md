# Topic Reference

The default topic names are also captured in:

```text
src/simbiosys_bringup/config/real_robot_topics.yaml
src/simbiosys_bringup/config/simulation_topics.yaml
```

| Purpose | Topic or Action |
| --- | --- |
| Base velocity | `/mirte_base_controller/cmd_vel` |
| Joint states | `/joint_states` |
| Arm trajectory | `/mirte_master_arm_controller/joint_trajectory` |
| Arm FollowJointTrajectory action | `/mirte_master_arm_controller/follow_joint_trajectory` |
| Gripper action | `/mirte_master_gripper_controller/gripper_cmd` |
| Laser scan | `/scan` |
| Odometry | `/mirte_base_controller/odom` |
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
ros2 topic echo /mirte_base_controller/odom --once
ros2 topic echo /camera/color/image_raw --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 action list
```

The current MIRTE Master odometry message uses `frame_id: odom` and
`child_frame_id: base_link`, so the `slam_toolbox` frame names remain standard
even though the odometry topic itself is controller-scoped.

Real robot frame names observed in May 2026:

- `/scan` uses `frame_id: laser`.
- `/camera/color/image_raw` uses `frame_id: camera_color_optical_frame`.
- `/camera/depth/image_raw` uses `frame_id: camera_depth_optical_frame`.
- The camera driver publishes camera-internal static frames under
  `camera_link`; verify that `base_link -> camera_link` is present before
  relying on camera data in map/base coordinates.

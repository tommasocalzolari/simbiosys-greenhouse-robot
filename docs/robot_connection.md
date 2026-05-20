# Robot Connection

Real robot mode is laptop-side only. Start low-level MIRTE bringup on the robot,
then run SimBioSys nodes on the laptop.

On the robot:

```bash
ssh mirte@<robot-ip>
source /opt/ros/humble/setup.bash
ros2 launch mirte_bringup minimal_master.launch.py
```

If the arm wrapper suddenly reports no subscribers, restart this robot-side
bringup first. The laptop wrapper only publishes trajectories; it does not
create the arm controller.

On the laptop:

```bash
pixi shell
source install/setup.bash
ros2 launch simbiosys_bringup laptop_system.launch.py
```

Verify discovery and topics:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once
ros2 topic echo /mirte_base_controller/odom --once
ros2 topic echo /camera/color/image_raw --once
ros2 action list
```

Verify TF without RViz or other visualization packages:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
```

The real robot has been observed publishing `/scan` with `frame_id: laser`,
color images with `camera_color_optical_frame`, and depth images with
`camera_depth_optical_frame`. If `odom -> base_link` works but sensor transforms
do not, check robot URDF/static transform bringup before debugging SLAM, Nav2,
or perception code.

Do not launch low-level robot bringup from `simbiosys_bringup`.

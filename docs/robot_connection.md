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

Do not launch low-level robot bringup from `simbiosys_bringup`.

# Robot Connection

Real robot mode is laptop-side only. Start low-level MIRTE bringup on the robot,
then run SimBioSys nodes on the laptop.

On the robot:

```bash
ssh mirte@<robot-ip>
ros2 launch mirte_bringup minimal_master.launch.py
```

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
ros2 topic echo /odom --once
```

Do not launch low-level robot bringup from `simbiosys_bringup`.

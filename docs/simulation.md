# Simulation

We reuse the MIRTE Master Gazebo launch:

```bash
ros2 launch simbiosys_bringup simulation_mirte_master.launch.py
```

This wraps:

```bash
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.xml
```

If the launch file logs that `mirte_gazebo` is missing, install MIRTE simulation
packages through `repos.repos`, Pixi, and `rosdep`, then rebuild.

Optional MoveIt launch, when installed:

```bash
ros2 launch mirte_moveit_config mirte_moveit.launch.py use_sim_time:=True
```

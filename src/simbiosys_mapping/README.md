# simbiosys_mapping

This package contains the launch files, parameters, RViz configs, helper nodes,
maps, and annotations used for MIRTE mapping, localization, and navigation.

The default saved map is always:

```text
maps/mirte_map.yaml
maps/mirte_map.pgm
```

Run the commands below from the workspace root:

```bash
cd ~/ro47007_mirte_ws
pixi shell
source install/setup.bash
```

After code changes, rebuild:

```bash
colcon build --packages-up-to simbiosys_mapping
source install/setup.bash
```

On this Linux workspace, if a plugin built with Pixi's newer compiler fails to
load with a `CXXABI` error, rebuild the C++ plugin with the Ubuntu compiler:

```bash
CC=/usr/bin/gcc CXX=/usr/bin/g++ \
  colcon build --packages-select simbiosys_final_approach_controller --cmake-clean-cache
colcon build --packages-select simbiosys_mapping
source install/setup.bash
```

## Real Robot Connection

Use the same ROS domain on the robot and laptop. The MIRTE robot has normally
worked on domain `0`.

On the robot over SSH:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
sudo systemctl restart mirte-ros
ros2 topic list --no-daemon
```

On the laptop:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

Important topics you should see from the robot:

```text
/scan
/tf
/tf_static
/mirte_base_controller/odom
/mirte_base_controller/cmd_vel
```

If `ros2 topic list` looks wrong, also try:

```bash
ros2 topic list --no-daemon
```

## Mapping

Mapping uses `slam_toolbox` and saves the map into the workspace `maps/` folder.
Do not run localization or navigation at the same time as mapping.

Simulation with the default static-obstacle world:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true
```

Simulation with the greenhouse world:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true world:=$(ros2 pkg prefix simbiosys_mapping)/share/simbiosys_mapping/worlds/greenhouse_8_beds.world
```

Real robot mapping:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false odom_topic:=/mirte_base_controller/odom
```

Useful mapping options:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false start_rviz:=false
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false map_name:=my_test_map
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false auto_save_period:=10.0
```

Manual save while mapping is running:

```bash
ros2 service call /getmap_node/save_map std_srvs/srv/Trigger "{}"
```

By default, the map is also saved periodically and once more on shutdown.

## Map Annotation

Use this after mapping is finished. This mode only annotates poses; it does not
modify the map image or map YAML.

Start annotation:

```bash
ros2 launch simbiosys_mapping map_annotation.launch.py
```

In RViz:

1. Use `2D Pose Estimate` to place the home pose.
2. Use `2D Goal Pose` to place each checkpoint in visit order.
3. Each pose stores position and orientation.

Save annotations:

```bash
ros2 service call /map_annotation_node/save_annotations std_srvs/srv/Trigger "{}"
```

Undo the last checkpoint:

```bash
ros2 service call /map_annotation_node/undo_last_checkpoint std_srvs/srv/Trigger "{}"
```

Reset all annotations:

```bash
ros2 service call /map_annotation_node/reset_annotation std_srvs/srv/Trigger "{}"
```

Output file:

```text
maps/mirte_map_annotations.json
```

Do not use `map_post_processing.launch.py` unless you intentionally want to run
the older map-cleaning tool that can overwrite map files.

## Simple Navigation

Simple navigation starts the map server, AMCL localization, Nav2 planner,
controller, behavior tree navigator, and RViz. The default map depends on the
selected mode:

```text
simulation:=false  maps/mirte_map.yaml
simulation:=true   maps/mirte_map_sim.yaml
```

Real robot:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

Simulation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=true
```

In RViz:

1. Use `2D Pose Estimate` to manually tell AMCL where the robot starts.
2. Use `2D Goal Pose` to send one navigation goal.

### Final approach

Normal navigation still uses the rotation shim and DWB. When less than `0.80 m`
of path remains, the controller latches into a direct holonomic pose approach.
It commands forward, lateral, and angular velocity together from goal error in
`base_link`, so the robot can strafe while rotating.

The direct approach:

- stops each axis independently inside tolerance;
- tapers speed near the goal and applies deadband compensation farther away;
- predicts the robot footprint over the next `0.50 s` in the local costmap;
- resets the Nav2 progress checker whenever control mode changes;
- returns to DWB if the path grows beyond `1.00 m` or direct motion is blocked;
- stays suppressed after an obstruction until the path exceeds `1.00 m` or a
  new goal is received, preventing repeated switching near the threshold.

Tune the `controller_server.ros__parameters.FollowPath` values in
`config/nav2_navigation.yaml`. The initial limits are `0.03 m`, `5 degrees`,
`0.50 m/s` for both translation axes, and `1.00 rad/s` for rotation.

Useful checks:

```bash
ros2 topic echo /amcl_pose --once
ros2 action list | grep navigate
ros2 topic echo /mirte_base_controller/cmd_vel --once
```

If Nav2 says `odom` does not exist, check the robot odometry and TF:

```bash
ros2 topic info /mirte_base_controller/odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Checkpoint Navigation

Checkpoint navigation uses `maps/mirte_map_annotations.json`. It sends one Nav2
goal at a time. It does not plan the whole route in advance.

Terminal 1: start normal navigation first.

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

For simulation, use this instead:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=true
```

Set the initial pose manually in RViz with `2D Pose Estimate`.

Terminal 2: start checkpoint navigation.

```bash
ros2 launch simbiosys_mapping checkpoint_navigation.launch.py
```

If you want to use a different annotation file:

```bash
ros2 launch simbiosys_mapping checkpoint_navigation.launch.py annotations_file:=maps/other_annotations.json
```

Terminal 3: command the next checkpoint.

```bash
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: next}"
```

Repeat the `next` command after the robot reaches each checkpoint.

Other checkpoint commands:

```bash
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: status}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: cancel}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: skip}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: reset}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: reload}"
```

Watch checkpoint status:

```bash
ros2 topic echo /checkpoint_status
```

Expected route:

```text
home pose -> checkpoint 1 -> checkpoint 2 -> ... -> final checkpoint
```

The robot is placed manually at the home pose; it does not need to drive there
automatically before the first checkpoint command.

## Important Files

```text
launch/getmap.launch.py                  mapping
launch/map_annotation.launch.py          map annotation
launch/navigation.launch.py              simple Nav2 navigation
launch/checkpoint_navigation.launch.py   command-driven checkpoint navigation

config/slam_toolbox_mapping.yaml         SLAM parameters
config/amcl_localization.yaml            AMCL localization parameters
config/nav2_navigation.yaml              Nav2 planner/controller/costmap parameters

rviz/getmap.rviz                         mapping RViz layout
rviz/map_annotation.rviz                 annotation RViz layout
rviz/navigation.rviz                     navigation RViz layout

simbiosys_mapping/getmap_node.py         saves maps
simbiosys_mapping/map_annotation_node.py saves annotations
simbiosys_mapping/checkpoint_navigator_node.py sends checkpoint goals to Nav2

maps/mirte_map.yaml                      default map metadata
maps/mirte_map.pgm                       default map image
maps/mirte_map_annotations.json          home/checkpoint annotations
```

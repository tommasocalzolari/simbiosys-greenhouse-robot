# TO DO:
- provare prima i chekcpoint sul robot vero
- provare a connettere con strafing

# simbiosys_mapping

Mapping, localization, and navigation package for the SimBioSys MIRTE Master
workspace.

This package does not implement SLAM, AMCL, or path planning from scratch. It
wraps existing ROS 2 packages with MIRTE-specific launch files, parameters, RViz
layouts, and helper nodes.

Main external packages used:

- `slam_toolbox` for online SLAM mapping.
- Nav2 `map_server` for loading saved maps.
- Nav2 `amcl` for Monte Carlo localization.
- Nav2 planner/controller/BT navigator for autonomous goal navigation.
- Gazebo and RViz for simulation and visualization.

## Package Layout

```text
simbiosys_mapping/
  behavior_trees/
    nav2_tight_space_backup_bt_deadband.xml
  config/
    slam_toolbox_mapping.yaml
    amcl_localization.yaml
    nav2_navigation.yaml
  launch/
    checkpoint_navigation.launch.py
    getmap.launch.py
    localization.launch.py
    map_annotation.launch.py
    map_post_processing.launch.py
    navigation.launch.py
  rviz/
    getmap.rviz
    localization.rviz
    map_annotation.rviz
    navigation.rviz
  simbiosys_mapping/
    checkpoint_navigator_node.py
    getmap_node.py
    initial_pose_node.py
    map_annotation_node.py
    map_post_processor_node.py
    mapping_status_node.py
  worlds/
    static_obstacles.world
  package.xml
  setup.py
  setup.cfg
```

Important files:

| File | Purpose |
| --- | --- |
| `launch/checkpoint_navigation.launch.py` | Starts the command-driven checkpoint navigator that sends one Nav2 goal at a time. |
| `launch/getmap.launch.py` | Starts SLAM mapping with `slam_toolbox`, optional Gazebo, RViz, and map auto-save. |
| `launch/localization.launch.py` | Starts standalone AMCL localization on a saved map. |
| `launch/map_annotation.launch.py` | Opens a saved map read-only and records home/checkpoint annotations. |
| `launch/map_post_processing.launch.py` | Older cleanup tool that modifies the saved map. Do not use it for annotation-only work. |
| `launch/navigation.launch.py` | Starts map server, AMCL, Nav2 planner/controller/BT navigator, optional Gazebo, and RViz. |
| `config/slam_toolbox_mapping.yaml` | SLAM Toolbox mapping parameters. |
| `config/amcl_localization.yaml` | AMCL and map-server parameters. |
| `config/nav2_navigation.yaml` | Nav2 behavior tree, planner, controller, behavior server, and costmap parameters. |
| `behavior_trees/nav2_tight_space_backup_bt_deadband.xml` | Custom Nav2 behavior tree with local clear and backup recovery. |
| `simbiosys_mapping/checkpoint_navigator_node.py` | Reads map annotations and advances through checkpoints using `/checkpoint_commands`. |
| `simbiosys_mapping/getmap_node.py` | Saves `/map` as `.yaml` and `.pgm` files in the workspace `maps/` folder. |
| `simbiosys_mapping/initial_pose_node.py` | Optional scripted AMCL initial-pose publisher. Navigation does not use it by default. |
| `simbiosys_mapping/map_annotation_node.py` | Annotation-only node. It never writes map YAML/PGM files. |
| `simbiosys_mapping/map_post_processor_node.py` | Older offline saved-map cleanup node that can overwrite map files. |
| `worlds/static_obstacles.world` | Gazebo world for repeatable mapping/localization/navigation tests. |

Extra debugging documents:

- [`docs/slam_debug_tuning.md`](../../docs/slam_debug_tuning.md)
- [`docs/localization_debug_tuning.md`](../../docs/localization_debug_tuning.md)
- [`docs/topic_reference.md`](../../docs/topic_reference.md)
- [`docs/robot_connection.md`](../../docs/robot_connection.md)

## ROS Graph Concept

Mapping, localization, and navigation are separate operating modes.

Mapping:

```text
robot/simulation -> /scan + /odom + /tf
slam_toolbox -> /map + map -> odom
getmap_node -> maps/<map_name>.yaml and maps/<map_name>.pgm
```

Localization:

```text
robot/simulation -> /scan + /odom + /tf
map_server -> /map
amcl -> /amcl_pose + /particle_cloud + map -> odom
```

Navigation:

```text
map_server + amcl -> localized robot pose in map
planner_server -> /plan
controller_server -> /mirte_base_controller/cmd_vel
bt_navigator -> goal execution and recovery behavior
```

Do not run mapping and localization/navigation at the same time. During mapping,
`slam_toolbox` owns `map -> odom`. During localization/navigation, AMCL owns
`map -> odom`.

Expected TF chain:

```text
map -> odom -> base_link -> lidar_base -> lidar_link
```

## Build

From the workspace root:

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --packages-select simbiosys_mapping
source install/setup.bash
```

If dependencies changed:

```bash
colcon build --packages-up-to simbiosys_mapping
source install/setup.bash
```

The package installs launch, config, RViz, world, documentation, and behavior
tree files into:

```text
install/share/simbiosys_mapping/
```

## Robot Connection

For the physical robot, the laptop and robot must be in the same ROS domain.
The robot has been observed to publish on domain `0`.

On the laptop:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

You should see robot topics such as:

```text
/scan
/tf
/tf_static
/mirte_base_controller/odom
/mirte_base_controller/cmd_vel
```

## Lidar TF Correction

The newer MIRTE Master needs a corrected lidar y offset. The old transform was
approximately:

```text
frame_link -> lidar_base: y = -0.1005
```

The corrected transform is approximately:

```text
frame_link -> lidar_base: y = -0.0005
```

Check it with:

```bash
ros2 run tf2_ros tf2_echo frame_link lidar_base
```

Expected output:

```text
Translation: [0.000, -0.001, 0.007]
```

The local source change is in:

```text
src/mirte-ros-packages/mirte_description/mirte_master_description/urdf/lidar.xacro
```

The same correction must also be deployed on the physical robot, because the
robot publishes its own `/tf_static`. If the robot still reports `y = -0.101`,
mapping, localization, and local costmaps can show self-hits or obstacles in the
wrong place.

After applying the correction, create a new map. Old maps made with the wrong
lidar TF are not reliable.

## Saved Maps

Maps are saved outside the package in the workspace-level `maps/` folder.

Default output:

```text
~/ro47007_mirte_ws/maps/mirte_map.yaml
~/ro47007_mirte_ws/maps/mirte_map.pgm
```

Run launch commands from the workspace root so relative map paths resolve
correctly:

```bash
cd ~/ro47007_mirte_ws
```

## Map Annotation

Use this when you only want to annotate home and checkpoints. This launch does
not modify `maps/mirte_map.yaml` or `maps/mirte_map.pgm`.

```bash
ros2 launch simbiosys_mapping map_annotation.launch.py
```

By default this reads `maps/mirte_map.yaml`, publishes it on `/map`, opens RViz,
and saves annotations to `maps/mirte_map_annotations.json`.

Annotation order in RViz:

1. `2D Pose Estimate` for the home pose.
2. `2D Goal Pose` for each checkpoint in the order the robot should visit them.

Finish and save the annotations:

```bash
ros2 service call /map_annotation_node/finish_annotation std_srvs/srv/Trigger "{}"
```

Default annotation file:

```text
maps/mirte_map_annotations.json
```

If you want to process another saved map:

```bash
ros2 launch simbiosys_mapping map_annotation.launch.py \
  map_yaml:=maps/other_map.yaml
```

The older `map_post_processing.launch.py` still exists, but it cleans and
overwrites the map. Do not use it if you only want annotations.

## Checkpoint Navigation

Checkpoint navigation uses `maps/mirte_map_annotations.json` and Nav2. It does
not create one long path through every checkpoint. It sends one `NavigateToPose`
goal only after a command is received.

Expected order:

```text
home pose -> checkpoint_1 -> checkpoint_2 -> ...
```

The robot should already be placed at the home pose and localized before the
first command. The node does not navigate to home.

Terminal 1, start normal navigation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

Terminal 2, start checkpoint navigation:

```bash
ros2 launch simbiosys_mapping checkpoint_navigation.launch.py
```

Terminal 3, send one checkpoint command at a time:

```bash
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: next}"
```

After the robot arrives, send `next` again for the next checkpoint.

Useful commands:

```bash
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: status}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: cancel}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: skip}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: reset}"
ros2 topic pub --once /checkpoint_commands std_msgs/msg/String "{data: reload}"
```

Status is published as JSON text on:

```text
/checkpoint_status
```

Inspect it with:

```bash
ros2 topic echo /checkpoint_status
```

## SLAM Mapping

Simulation mapping:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true
```

`simulation:=true` is the default:

```bash
ros2 launch simbiosys_mapping getmap.launch.py
```

Real robot mapping:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false
```

Real robot mapping expects:

```text
/scan
/odom or /mirte_base_controller/odom
/tf
/tf_static
```

If needed, pass topic names explicitly:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  scan_topic:=/scan \
  odom_topic:=/mirte_base_controller/odom
```

### Mapping Workflow

Use this process for cleaner maps:

1. Verify the lidar TF correction first.
2. Place the robot parallel to a long wall before starting mapping.
3. Start mapping and keep the robot still for a few seconds.
4. Drive slowly.
5. Avoid sharp turns at the start.
6. Drive loops and revisit earlier places so loop closure can correct drift.
7. Stop mapping only after the map has been auto-saved or saved manually.

If the whole map is rotated but walls are straight, the robot probably started
with a rotated heading. That is mostly a visualization issue. If walls bend or
shear, check odometry, TF, scan quality, and the SLAM tuning guide.

### Map Auto-Save

`getmap_node` saves the latest `/map` every 20 seconds once a map has arrived.

Change map name:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  map_name:=test_map
```

Change auto-save period:

```bash
ros2 launch simbiosys_mapping getmap.launch.py auto_save_period:=5.0
```

Disable periodic auto-save:

```bash
ros2 launch simbiosys_mapping getmap.launch.py auto_save_period:=0.0
```

Save manually:

```bash
ros2 service call /getmap_node/save_map std_srvs/srv/Trigger "{}"
```

Show mapping launch arguments:

```bash
ros2 launch simbiosys_mapping getmap.launch.py --show-args
```

## Monte Carlo Localization

Standalone localization uses a saved map and AMCL. It does not build a new map.

Real robot:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

Simulation:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=true \
  map:=maps/mirte_map.yaml
```

After RViz opens, use `2D Pose Estimate` to set the robot's initial pose. AMCL
then publishes:

```text
/amcl_pose
/particle_cloud
map -> odom
```

Automatic initial pose publishing is available for scripted tests, but it is off
by default. Normal operation should use manual RViz initialization.

Show localization launch arguments:

```bash
ros2 launch simbiosys_mapping localization.launch.py --show-args
```

## Nav2 Navigation

Navigation is self-contained. It starts:

- map server loading `maps/mirte_map.yaml`
- AMCL localization
- Nav2 controller server
- Nav2 planner server
- Nav2 behavior server
- Nav2 BT navigator
- RViz
- optional Gazebo when `simulation:=true`

Run real robot navigation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

Run simulation navigation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=true
```

The navigation launch always loads:

```text
maps/mirte_map.yaml
```

After RViz opens:

1. Set the initial pose with `2D Pose Estimate`.
2. Wait for AMCL to align the scan with the map.
3. Send a `2D Goal Pose`.

Navigation intentionally does not start `initial_pose_node`; the operator should
manually set the initial pose.

### Navigation Topics

Inputs:

```text
/scan
/mirte_base_controller/odom
/tf
/tf_static
/goal_pose
```

Main outputs:

```text
/plan
/local_costmap/costmap
/global_costmap/costmap
/local_costmap/published_footprint
/global_costmap/published_footprint
/mirte_base_controller/cmd_vel
```

Command topic selection:

| Mode | Command topic |
| --- | --- |
| `simulation:=true` | `/mirte_base_controller/cmd_vel_unstamped` |
| `simulation:=false` | `/mirte_base_controller/cmd_vel` |

Override the command topic only if needed:

```bash
ros2 launch simbiosys_mapping navigation.launch.py \
  simulation:=false \
  cmd_vel_topic:=/your_cmd_vel_topic
```

Show navigation launch arguments:

```bash
ros2 launch simbiosys_mapping navigation.launch.py --show-args
```

### Current Navigation Design

The current Nav2 setup is tuned for the MIRTE robot and a small indoor map.

Planner:

- `nav2_smac_planner/SmacPlanner2D`
- `cost_travel_multiplier` is used to prefer lower-cost cells and safer paths.
- The global costmap uses the static map plus inflation. It does not currently
  add live laser obstacles to the global costmap.

Controller:

- `nav2_rotation_shim_controller::RotationShimController`
- Primary controller is `dwb_core::DWBLocalPlanner`.
- Rotation shim is used so the robot first aligns with the path, then drives.
- Angular velocity settings account for the observed MIRTE angular deadband of
  about `0.4 rad/s`.

Costmaps:

- The global costmap is more conservative and is used to choose safer routes.
- The local costmap is smaller and more permissive so the robot does not freeze
  as easily.
- New obstacles seen only by the local costmap are handled locally. If they
  block the route completely, the behavior tree should clear/recover/replan
  rather than expecting the controller to invent a new global route.

Behavior tree:

- `behavior_trees/nav2_tight_space_backup_bt_deadband.xml`
- Installed into `share/simbiosys_mapping/behavior_trees/`.
- `navigation.launch.py` rewrites the BT path at launch time so the YAML is
  portable and does not contain a laptop-specific absolute path.
- The tree computes a path, follows it, clears local/global costmaps when needed,
  backs up in tight spaces, spins, waits, and retries.

Check which BT is active:

```bash
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

Expected value should contain:

```text
nav2_tight_space_backup_bt_deadband.xml
```

### Navigation Debug Checks

Before sending a goal:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /mirte_base_controller/cmd_vel
```

If the robot does not move:

```bash
ros2 action info /navigate_to_pose
ros2 topic echo /plan --once
ros2 topic echo /mirte_base_controller/cmd_vel --once
ros2 topic info /mirte_base_controller/cmd_vel
```

For simulation, use:

```bash
ros2 topic echo /mirte_base_controller/cmd_vel_unstamped --once
ros2 topic info /mirte_base_controller/cmd_vel_unstamped
```

Interpretation:

| Symptom | Likely issue |
| --- | --- |
| `/bt_navigator` is not `active` | Nav2 lifecycle did not activate. Check the launch terminal. |
| `/plan` is empty | Planner cannot find a route or the goal is in occupied/unknown space. |
| `/plan` exists but no velocity appears | Controller cannot follow the path. Check local costmap, footprint, and controller logs. |
| Velocity publishes but topic has no subscriber | Nav2 is publishing to the wrong command topic. |
| Velocity publishes and has a subscriber | Nav2 is commanding motion; check base controller, motor enable, or safety state. |

Clear the local costmap during testing:

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

If the robot moves after clearing, the issue is likely stale local obstacles,
self-hits, or a footprint/costmap mismatch.

## RViz Visualization

Useful displays:

```text
/map
/scan
/amcl_pose
/particle_cloud
/plan
/local_costmap/costmap
/global_costmap/costmap
/local_costmap/published_footprint
/global_costmap/published_footprint
```

The costmap footprint topics show the collision footprint Nav2 uses. The robot
model display shows the URDF mesh, which may not exactly match the Nav2 collision
radius.

## Manual Teleop

Drive in simulation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Drive slowly while mapping. Fast motion and sharp turns can make SLAM and AMCL
look worse than the underlying parameters.

## Lingering Process Checks

On the robot:

```bash
ros2 node list
ps aux | grep -E "ros2|launch|slam|nav2|amcl|map_server|robot_state_publisher" | grep -v grep
sudo systemctl status mirte-ros --no-pager
```

Restart the MIRTE ROS service if needed:

```bash
sudo service mirte-ros restart
```

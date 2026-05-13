# simbiosys_mapping

Mapping and localization package for the SimBioSys MIRTE workspace.

This package does not implement a SLAM or localization algorithm from scratch.
It reuses existing ROS 2 packages:

- `slam_toolbox` for SLAM mapping.
- Nav2 `map_server` for loading a saved map.
- Nav2 `amcl` for Monte Carlo localization.
- Nav2 `lifecycle_manager` for activating localization nodes.

The package adds SimBioSys-specific launch files, configuration, RViz layouts,
a small map-saving helper node, and a Gazebo world with static obstacles.

## Package Layout

```text
simbiosys_mapping/
  config/
    slam_toolbox_mapping.yaml
    amcl_localization.yaml
  launch/
    getmap.launch.py
    localization.launch.py
  rviz/
    getmap.rviz
    localization.rviz
  simbiosys_mapping/
    getmap_node.py
    initial_pose_node.py
    mapping_status_node.py
  worlds/
    static_obstacles.world
  package.xml
  setup.py
  setup.cfg
```

Main files:

- `launch/getmap.launch.py`: SLAM mapping launch.
- `launch/localization.launch.py`: Monte Carlo localization launch.
- `config/slam_toolbox_mapping.yaml`: `slam_toolbox` mapping parameters.
- `config/amcl_localization.yaml`: Nav2 AMCL localization parameters.
- `simbiosys_mapping/getmap_node.py`: watches mapping topics and saves maps.
- `simbiosys_mapping/initial_pose_node.py`: optionally publishes AMCL's initial pose.
- `worlds/static_obstacles.world`: Gazebo world for simulation tests.
- `rviz/getmap.rviz`: RViz layout for building a map.
- `rviz/localization.rviz`: RViz layout for localizing in a saved map.

## Important Concept

Mapping and localization are separate modes.

Mapping:

```text
robot/simulation -> /scan + /odom + /tf
slam_toolbox -> /map + map -> odom
getmap_node -> saves maps/mirte_map.yaml and maps/mirte_map.pgm
```

Localization:

```text
robot/simulation -> /scan + /odom + /tf
map_server -> loads maps/mirte_map.yaml
amcl -> /amcl_pose + /particle_cloud + map -> odom
```

Do not run `getmap.launch.py` and `localization.launch.py` at the same time.
Both mapping and localization try to own the `map -> odom` transform.

## Build

From the workspace root:

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --packages-select simbiosys_mapping
source install/setup.bash
```

If dependencies changed, build up to this package:

```bash
colcon build --packages-up-to simbiosys_mapping
source install/setup.bash
```

## Saved Map Location

Maps are saved outside the package, in the workspace-level `maps/` folder.

Default mapping output:

```text
~/ro47007_mirte_ws/maps/mirte_map.yaml
~/ro47007_mirte_ws/maps/mirte_map.pgm
```

Run launch commands from the workspace root so the relative `maps/...` path is
resolved correctly:

```bash
cd ~/ro47007_mirte_ws
```

You can also pass absolute map paths if needed.

## SLAM Mapping

Simulation mode starts Gazebo with the static-obstacle world, starts
`slam_toolbox`, starts `getmap_node`, and opens RViz:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true
```

`simulation:=true` is the default:

```bash
ros2 launch simbiosys_mapping getmap.launch.py
```

Real robot mapping does not start Gazebo:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false
```

The robot must already publish:

```text
/scan
/odom
/tf
/tf_static
```

If the real robot uses different topic names:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  scan_topic:=/your_scan_topic \
  odom_topic:=/your_odom_topic
```

### Mapping Auto-Save

`getmap_node` saves the latest `/map` every 20 seconds once a map has arrived.

Default output:

```text
maps/mirte_map.yaml
maps/mirte_map.pgm
```

Change output:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  output_dir:=maps \
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

### Mapping Launch Options

Show all arguments:

```bash
ros2 launch simbiosys_mapping getmap.launch.py --show-args
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `simulation` | `true` | `true` starts Gazebo; `false` uses real robot topics only. |
| `gazebo_gui` | `true` | Show or hide Gazebo GUI in simulation mode. |
| `world` | `worlds/static_obstacles.world` | Gazebo world used when `simulation:=true`. |
| `start_rviz` | `true` | Start RViz. |
| `rviz_config` | `rviz/getmap.rviz` | RViz config file. |
| `use_sim_time` | `auto` | `auto` follows `simulation`; can be forced to `true` or `false`. |
| `slam_params_file` | `config/slam_toolbox_mapping.yaml` | Parameters for `slam_toolbox`. |
| `scan_topic` | `/scan` | Laser scan topic. |
| `odom_topic` | `/odom` | Odometry topic monitored by `getmap_node`. |
| `map_topic` | `/map` | Occupancy grid topic from `slam_toolbox`. |
| `output_dir` | `maps` | Directory where map files are saved. |
| `map_name` | `mirte_map` | Base name for `.yaml` and `.pgm` map files. |
| `auto_save_period` | `20.0` | Seconds between automatic saves. `0.0` disables periodic saving. |
| `save_on_shutdown` | `true` | Save once when `getmap_node` shuts down, if a map exists. |

## Monte Carlo Localization

Localization uses a previously saved map and AMCL. It does not build a new map.

Real robot localization:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

Simulation localization:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=true \
  map:=maps/mirte_map.yaml  
```

In simulation mode, the same `worlds/static_obstacles.world` is used by
default. This only makes sense if the saved map was created from the same world
or from a compatible environment.

After RViz opens, use the `2D Pose Estimate` tool to give AMCL the robot's
initial pose on the map. AMCL then publishes:

```text
/amcl_pose
/particle_cloud
map -> odom
```

In simulation mode, `localization.launch.py` automatically publishes an initial
pose at `(0, 0, 0)` a few times. This helps AMCL start publishing `map -> odom`
so RViz gets a valid `map` frame. If the pose is not accurate, reset it in RViz
with `2D Pose Estimate`.

For the real robot, automatic initial pose publishing is off by default. Use
RViz `2D Pose Estimate`, or enable it explicitly if you know the robot's pose:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml \
  publish_initial_pose:=true \
  initial_pose_x:=0.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0
```

### Localization Launch Options

Show all arguments:

```bash
ros2 launch simbiosys_mapping localization.launch.py --show-args
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `simulation` | `false` | `true` starts Gazebo; `false` uses real robot topics only. |
| `gazebo_gui` | `true` | Show or hide Gazebo GUI in simulation mode. |
| `world` | `worlds/static_obstacles.world` | Gazebo world used when `simulation:=true`. |
| `start_rviz` | `true` | Start RViz. |
| `rviz_config` | `rviz/localization.rviz` | RViz config file. |
| `use_sim_time` | `auto` | `auto` follows `simulation`; can be forced to `true` or `false`. |
| `map` | `maps/mirte_map.yaml` | Saved map YAML loaded by Nav2 map server. |
| `params_file` | `config/amcl_localization.yaml` | AMCL/map server/lifecycle parameters. |
| `scan_topic` | `/scan` | Laser scan topic used by AMCL. |
| `autostart` | `true` | Automatically activate `map_server` and `amcl`. |
| `publish_initial_pose` | `auto` | `auto` publishes initial pose only in simulation; can be forced to `true` or `false`. |
| `initial_pose_x` | `0.0` | Initial AMCL pose x in the map frame. |
| `initial_pose_y` | `0.0` | Initial AMCL pose y in the map frame. |
| `initial_pose_yaw` | `0.0` | Initial AMCL yaw in radians. |
| `initial_pose_period` | `1.0` | Seconds between repeated initial-pose messages. |
| `initial_pose_count` | `10` | Number of initial-pose messages to publish. |

## Useful Checks

For mapping:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

For localization:

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /particle_cloud --once
ros2 run tf2_ros tf2_echo map odom
```

For the full TF tree:

```bash
ros2 run tf2_tools view_frames
```

Expected TF chain:

```text
map -> odom -> base_link -> lidar_link
```

If `/scan` is missing, the laser or simulation is not publishing.

If `/odom` is missing, robot odometry or the base controller is not publishing.

If `/map` is missing during mapping, check `slam_toolbox` and TF.

If `/map` is missing during localization, check the `map:=...` path.

If `/amcl_pose` is missing, check AMCL startup and set the initial pose in RViz.

If RViz says `Fixed Frame [map] does not exist`, it usually means AMCL has not
published `map -> odom` yet. Make sure `/map` exists, then set an initial pose in
RViz or enable `publish_initial_pose:=true`.

## Manual Teleop

To drive while mapping or localizing in simulation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Drive slowly while mapping so the map updates cleanly.

# simbiosys_mapping Integration README

This document is for the person integrating or merging the mapping,
localization, and navigation work into another branch or workspace.

For now, transfer the complete `src/simbiosys_mapping/` package and the related
documentation files listed below. Some helper nodes could be simplified later,
but at this stage keep all files together so the launch files, package install
rules, RViz configs, and console scripts still match.

## What This Package Provides

`simbiosys_mapping` does not implement SLAM, AMCL, or Nav2 from scratch. It wraps
existing ROS 2 packages with project-specific configuration:

- `slam_toolbox` for building a map.
- Nav2 `map_server` for loading a saved map.
- Nav2 `amcl` for localization in a saved map.
- Nav2 planner/controller/BT navigator for goal navigation.
- Gazebo/RViz launch support for simulation testing.
- Small Python helper nodes for saving maps and optionally publishing an initial
  pose.

## Files To Transfer

Transfer the full package:

```text
src/simbiosys_mapping/
  README.md
  INTEGRATION_README.md
  package.xml
  setup.py
  setup.cfg
  resource/simbiosys_mapping
  config/
    slam_toolbox_mapping.yaml
    amcl_localization.yaml
    nav2_navigation.yaml
  launch/
    getmap.launch.py
    localization.launch.py
    navigation.launch.py
  rviz/
    getmap.rviz
    localization.rviz
    navigation.rviz
  simbiosys_mapping/
    __init__.py
    getmap_node.py
    initial_pose_node.py
    mapping_status_node.py
  worlds/
    static_obstacles.world
```

Do not transfer generated cache files:

```text
__pycache__/
*.pyc
```

Also transfer the mapping/localization documentation:

```text
docs/mapping.md
docs/slam_debug_tuning.md
docs/localization_debug_tuning.md
docs/topic_reference.md
docs/README.md
```

Saved maps are intentionally outside the package. If localization/navigation
should work immediately in the merged workspace, also transfer the selected map:

```text
maps/mirte_map.yaml
maps/mirte_map.pgm
```

or whichever `.yaml`/`.pgm` map pair is currently used.

## Package Metadata

Keep these files:

| File | Why it is needed |
| --- | --- |
| `package.xml` | Declares ROS dependencies such as `slam_toolbox`, `nav2_amcl`, `nav2_bt_navigator`, `nav2_controller`, `nav2_map_server`, `dwb_core`, and RViz. |
| `setup.py` | Installs Python nodes, launch files, config files, RViz files, worlds, and package documentation. |
| `setup.cfg` | Tells ROS where Python console scripts are installed. |
| `resource/simbiosys_mapping` | Required by `ament_python` so ROS can find the package. |
| `simbiosys_mapping/__init__.py` | Required so the Python node folder is a Python package. |

## Launch Files

### `launch/getmap.launch.py`

Mapping launch. It starts `slam_toolbox`, the map-saving node, RViz, and
optionally Gazebo.

Main mode flag:

```bash
simulation:=true   # Gazebo + mapping
simulation:=false  # real robot + mapping
```

Important inputs:

```text
/scan
/odom
/tf
/tf_static
```

Important outputs:

```text
/map
map -> odom TF
maps/<map_name>.yaml
maps/<map_name>.pgm
```

Default saved map path:

```text
maps/mirte_map.yaml
maps/mirte_map.pgm
```

### `launch/localization.launch.py`

Localization launch. It starts Nav2 `map_server`, Nav2 `amcl`, lifecycle
management, RViz, and optionally Gazebo.

Main mode flag:

```bash
simulation:=true   # Gazebo + AMCL on saved map
simulation:=false  # real robot + AMCL on saved map
```

Important inputs:

```text
map:=maps/mirte_map.yaml
/scan
/odom
/tf
/tf_static
/initialpose
```

Important outputs:

```text
/map
/amcl_pose
/particle_cloud
map -> odom TF
```

Initial pose is manual by default. The operator should set it in RViz with
`2D Pose Estimate`.

### `launch/navigation.launch.py`

Nav2 navigation launch. This assumes localization is already running and AMCL
has already published `map -> odom`.

Important inputs:

```text
/goal_pose
/map
/scan
/mirte_base_controller/odom
/tf
/tf_static
```

Important outputs:

```text
/plan
/local_plan
/global_costmap/costmap
/local_costmap/costmap
/mirte_base_controller/cmd_vel              # real robot
/mirte_base_controller/cmd_vel_unstamped    # simulation
```

The launch file uses:

```text
cmd_vel_topic:=auto
```

which maps the command topic as:

| Mode | Command topic |
| --- | --- |
| `simulation:=true` | `/mirte_base_controller/cmd_vel_unstamped` |
| `simulation:=false` | `/mirte_base_controller/cmd_vel` |

If the merged robot stack uses a different command topic, override it:

```bash
ros2 launch simbiosys_mapping navigation.launch.py \
  simulation:=false \
  cmd_vel_topic:=/your_robot/cmd_vel
```

## Config Files

| File | Used by | Purpose |
| --- | --- | --- |
| `config/slam_toolbox_mapping.yaml` | `getmap.launch.py` | SLAM Toolbox frames, scan matching, map resolution, update intervals, loop closure, laser range limits. |
| `config/amcl_localization.yaml` | `localization.launch.py` | AMCL particle filter, laser model, odometry noise, frames, map server lifecycle settings. |
| `config/nav2_navigation.yaml` | `navigation.launch.py` | Nav2 BT navigator, planner, controller, behavior server, global/local costmaps. |

Expected frame chain:

```text
map -> odom -> base_link -> lidar frame
```

`slam_toolbox` owns `map -> odom` during mapping. `amcl` owns `map -> odom`
during localization/navigation. Do not run mapping and localization at the same
time.

## RViz Files

| File | Purpose |
| --- | --- |
| `rviz/getmap.rviz` | Mapping view: map, scan, robot model, TF. |
| `rviz/localization.rviz` | Localization view: saved map, AMCL pose, particles, scan, robot. |
| `rviz/navigation.rviz` | Navigation view: map, global/local costmaps, plan, scan, AMCL pose, Nav2 panel. |

## Python Nodes

Keep these for now, even if some could later be replaced by standard commands.

| Node file | Console script | Reads | Publishes / provides | Purpose |
| --- | --- | --- | --- | --- |
| `simbiosys_mapping/getmap_node.py` | `getmap_node` | `/map`, `/scan`, `/odom` | `/getmap_node/save_map` service, `.yaml/.pgm` files | Saves the occupancy grid to the workspace `maps/` folder automatically or on service call. |
| `simbiosys_mapping/initial_pose_node.py` | `initial_pose_node` | parameters only | `/initialpose` | Optional scripted AMCL initial pose publisher. Disabled by default in normal use. |
| `simbiosys_mapping/mapping_status_node.py` | `mapping_status_node` | `/map`, `/scan`, `/odom` | log output | Small status/debug helper for checking whether mapping topics are alive. |

## Gazebo World

`worlds/static_obstacles.world` is the simulation world used by mapping and
localization when:

```bash
simulation:=true
```

It gives a repeatable static-obstacle environment for SLAM/localization tests.

## Runtime Workflow

Mapping:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true
```

Localization:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=true \
  map:=maps/mirte_map.yaml
```

Navigation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=true
```

For the real robot, use `simulation:=false` and make sure the laptop can see
robot topics:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

## Integration Checks

After merging, run:

```bash
colcon build --packages-select simbiosys_mapping
source install/setup.bash
ros2 launch simbiosys_mapping getmap.launch.py --show-args
ros2 launch simbiosys_mapping localization.launch.py --show-args
ros2 launch simbiosys_mapping navigation.launch.py --show-args
```

Topic checks for mapping/localization:

```bash
ros2 topic echo /scan --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

Topic checks for navigation:

```bash
ros2 lifecycle get /bt_navigator
ros2 topic echo /plan --once
ros2 topic echo /mirte_base_controller/cmd_vel --once
ros2 topic info /mirte_base_controller/cmd_vel
```

Use `/mirte_base_controller/cmd_vel_unstamped` instead of
`/mirte_base_controller/cmd_vel` in simulation.


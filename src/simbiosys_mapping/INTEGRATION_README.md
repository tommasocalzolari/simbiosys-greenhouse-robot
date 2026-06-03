# simbiosys_mapping Integration README

This document is for the person merging the mapping, localization, and
navigation work into another branch or workspace.

Transfer the complete `src/simbiosys_mapping/` package. Some helper nodes could
be simplified later, but for now keep the package intact so launch files,
install rules, behavior trees, RViz configs, and console scripts stay
consistent.

## What This Package Provides

`simbiosys_mapping` wraps existing ROS 2 packages for the MIRTE Master:

- `slam_toolbox` for SLAM mapping.
- Nav2 `map_server` and `amcl` for localization.
- Nav2 planner, controller, behavior server, and BT navigator for goal
  navigation.
- A custom Nav2 behavior tree for tight-space recovery.
- RViz configs for mapping, localization, and navigation.
- Gazebo static-obstacle world for simulation tests.
- Python helper nodes for map saving, optional initial-pose publishing, and
  mapping status checks.

It does not implement SLAM, AMCL, or planning algorithms from scratch.

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
  behavior_trees/
    nav2_tight_space_backup_bt_deadband.xml
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

Do not transfer generated files:

```text
__pycache__/
*.pyc
```

Also transfer the relevant documentation:

```text
docs/mapping.md
docs/slam_debug_tuning.md
docs/localization_debug_tuning.md
docs/topic_reference.md
docs/robot_connection.md
docs/README.md
```

Saved maps are intentionally outside the package. If localization/navigation
should work immediately, also transfer the selected map pair:

```text
maps/mirte_map.yaml
maps/mirte_map.pgm
```

## External Robot Description Change

This work also depends on a MIRTE Master lidar TF correction outside
`simbiosys_mapping`:

```text
src/mirte-ros-packages/mirte_description/mirte_master_description/urdf/lidar.xacro
```

The corrected `frame_link -> lidar_base` translation should be approximately:

```text
Translation: [0.000, -0.001, 0.007]
```

The old/wrong value was approximately:

```text
Translation: [0.000, -0.101, 0.007]
```

Verify on the real robot:

```bash
ros2 run tf2_ros tf2_echo frame_link lidar_base
```

The same correction must be present on the physical robot installation, because
the robot publishes its own `/tf_static`. If this fix is missing, mapping,
localization, and local costmaps can show obstacles in the wrong place.

After this correction, make a new map. Do not rely on maps created with the old
lidar transform.

## Package Install Rules

`simbiosys_mapping` is an `ament_python` package. Install rules are in
`setup.py`, not `CMakeLists.txt`.

`setup.py` installs:

- package metadata and READMEs
- YAML config files
- launch files
- RViz files
- Gazebo world files
- behavior tree XML files
- Python console scripts

The behavior tree XML must be installed to:

```text
install/share/simbiosys_mapping/behavior_trees/
```

This is already handled by:

```python
(
    os.path.join("share", package_name, "behavior_trees"),
    glob("behavior_trees/*.xml"),
)
```

## ROS Dependencies

Important runtime dependencies in `package.xml` include:

```text
slam_toolbox
nav2_amcl
nav2_map_server
nav2_lifecycle_manager
nav2_bt_navigator
nav2_behavior_tree
nav2_controller
nav2_planner
nav2_smac_planner
nav2_behaviors
nav2_costmap_2d
dwb_core
dwb_critics
rviz2
mirte_gazebo
```

## Launch Files

### `launch/getmap.launch.py`

Starts SLAM mapping.

Can run in simulation:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true
```

or on the real robot:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false
```

Main inputs:

```text
/scan
/odom or /mirte_base_controller/odom
/tf
/tf_static
```

Main outputs:

```text
/map
map -> odom
maps/<map_name>.yaml
maps/<map_name>.pgm
```

`getmap_node` saves the map periodically and exposes:

```text
/getmap_node/save_map
```

### `launch/localization.launch.py`

Starts standalone AMCL localization on a saved map.

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

Main inputs:

```text
map:=maps/mirte_map.yaml
/scan
/tf
/tf_static
/initialpose
```

Main outputs:

```text
/map
/amcl_pose
/particle_cloud
map -> odom
```

Initial pose is manual by default. The operator should use RViz `2D Pose
Estimate`.

### `launch/navigation.launch.py`

Starts the full navigation stack. It is self-contained and starts:

- `map_server`
- `amcl`
- `controller_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- RViz
- optional Gazebo when `simulation:=true`

Real robot:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

Simulation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=true
```

It always loads:

```text
maps/mirte_map.yaml
```

The operator must set the initial pose in RViz with `2D Pose Estimate` before
sending a goal. `navigation.launch.py` does not start `initial_pose_node`.

Command topic selection:

| Mode | Command topic |
| --- | --- |
| `simulation:=true` | `/mirte_base_controller/cmd_vel_unstamped` |
| `simulation:=false` | `/mirte_base_controller/cmd_vel` |

Override only if the merged base stack uses another command topic:

```bash
ros2 launch simbiosys_mapping navigation.launch.py \
  simulation:=false \
  cmd_vel_topic:=/your_cmd_vel_topic
```

## Navigation Configuration

`config/nav2_navigation.yaml` currently uses:

- `nav2_smac_planner/SmacPlanner2D` as the global planner.
- `nav2_rotation_shim_controller::RotationShimController` as the outer
  controller.
- `dwb_core::DWBLocalPlanner` as the primary local controller.
- A custom behavior tree for local clear, backup, spin, wait, and retry.

The planner is tuned to prefer safer paths using:

```text
cost_travel_multiplier
global_costmap inflation_radius
global_costmap cost_scaling_factor
```

The controller is tuned around observed MIRTE deadbands:

```text
minimum useful angular velocity: about 0.4 rad/s
minimum useful linear velocity: about 0.2 m/s
```

The global costmap uses the static map plus inflation. The local costmap uses
laser observations for nearby obstacles. A local obstacle can stop or recover
the robot, but the local controller should not be treated as a full global
replanner.

## Behavior Tree

The custom behavior tree is:

```text
behavior_trees/nav2_tight_space_backup_bt_deadband.xml
```

It:

- computes a path to the goal
- follows the path
- clears local costmap and backs up if local following fails
- clears global/local costmaps during recovery
- can spin or wait during recovery
- retries navigation
- accepts updated goals

`config/nav2_navigation.yaml` contains only a portable placeholder:

```yaml
default_nav_to_pose_bt_xml: nav2_tight_space_backup_bt_deadband.xml
```

`navigation.launch.py` rewrites that value at launch time to the installed
package path:

```text
install/share/simbiosys_mapping/behavior_trees/nav2_tight_space_backup_bt_deadband.xml
```

Verify while navigation is running:

```bash
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
```

Expected value should contain:

```text
nav2_tight_space_backup_bt_deadband.xml
```

## RViz Files

| File | Purpose |
| --- | --- |
| `rviz/getmap.rviz` | Mapping view: map, scan, robot model, TF. |
| `rviz/localization.rviz` | Localization view: saved map, AMCL pose, particles, scan, robot. |
| `rviz/navigation.rviz` | Navigation view: map, AMCL, plan, costmaps, scan, footprint, Nav2 tools. |

Important navigation displays:

```text
/plan
/local_costmap/costmap
/global_costmap/costmap
/local_costmap/published_footprint
/global_costmap/published_footprint
```

The footprint topics show Nav2's collision model. The robot model shows the URDF
mesh and may not exactly match the costmap radius.

## Python Nodes

Keep these for now:

| Node file | Console script | Purpose |
| --- | --- | --- |
| `simbiosys_mapping/getmap_node.py` | `getmap_node` | Saves `/map` to the workspace `maps/` folder automatically or on service call. |
| `simbiosys_mapping/initial_pose_node.py` | `initial_pose_node` | Optional scripted AMCL initial-pose publisher. Disabled in normal navigation use. |
| `simbiosys_mapping/mapping_status_node.py` | `mapping_status_node` | Small topic-status helper for mapping debug. |

## Runtime Workflow

Real robot connection:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

Mapping:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false
```

Localization only:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

Navigation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

## Integration Checks

After merging:

```bash
colcon build --packages-select simbiosys_mapping
source install/setup.bash
ros2 launch simbiosys_mapping getmap.launch.py --show-args
ros2 launch simbiosys_mapping localization.launch.py --show-args
ros2 launch simbiosys_mapping navigation.launch.py --show-args
```

Check installed behavior tree:

```bash
test -f install/share/simbiosys_mapping/behavior_trees/nav2_tight_space_backup_bt_deadband.xml
```

Check robot TF:

```bash
ros2 run tf2_ros tf2_echo frame_link lidar_base
```

Check localization/navigation topics:

```bash
ros2 topic echo /scan --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /bt_navigator
ros2 topic echo /plan --once
ros2 topic echo /mirte_base_controller/cmd_vel --once
```

Use `/mirte_base_controller/cmd_vel_unstamped` instead of
`/mirte_base_controller/cmd_vel` in simulation.

## Known Practical Notes

- Create a fresh map after the lidar TF fix.
- If the whole map is rotated but geometrically straight, the robot likely
  started mapping at a rotated heading.
- If the map bends or shears, debug odometry, TF, scan quality, and SLAM
  parameters.
- If the robot freezes near obstacles, inspect the local costmap and published
  footprint in RViz.
- If the robot hits new obstacles, the local costmap/controller tuning is the
  first place to inspect.
- If the robot does not move even though a goal is accepted, check whether
  `/cmd_vel` is being published and whether the base controller subscribes.

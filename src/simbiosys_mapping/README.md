# simbiosys_mapping

Mapping package for the SimBioSys MIRTE workspace.

This package does not implement a SLAM algorithm. It reuses `slam_toolbox` for
SLAM and adds SimBioSys-specific launch, configuration, visualization, a simple
map-saving node, and a Gazebo test world.

## What Is In This Package

```text
simbiosys_mapping/
  config/
    slam_toolbox_mapping.yaml
  launch/
    getmap.launch.py
  rviz/
    getmap.rviz
  simbiosys_mapping/
    getmap_node.py
    mapping_status_node.py
  worlds/
    static_obstacles.world
  package.xml
  setup.py
  setup.cfg
```

Important files:

- `config/slam_toolbox_mapping.yaml`: parameters for `slam_toolbox`.
- `launch/getmap.launch.py`: main mapping launch file.
- `simbiosys_mapping/getmap_node.py`: watches mapping topics and saves maps.
- `rviz/getmap.rviz`: RViz layout for watching `/map` and `/scan`.
- `worlds/static_obstacles.world`: Gazebo world with static obstacles.

## Mapping Workflow

The runtime graph is:

```text
MIRTE Gazebo or real robot
  publishes /scan, /odom, /tf

slam_toolbox
  subscribes to /scan and TF
  publishes /map and map -> odom TF

getmap_node
  subscribes to /scan, /odom, /map
  auto-saves the latest /map as .yaml + .pgm

rviz2
  displays /map, /scan, TF, and robot model
```

## Build

From the workspace root:

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --packages-select simbiosys_mapping
source install/setup.bash
```

If dependencies or other packages changed, build up to this package:

```bash
colcon build --packages-up-to simbiosys_mapping
source install/setup.bash
```

## Run In Simulation

This starts Gazebo, the static-obstacle world, `slam_toolbox`, `getmap_node`,
and RViz:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=true
```

`simulation:=true` is the default, so this is equivalent:

```bash
ros2 launch simbiosys_mapping getmap.launch.py
```

The simulation world defaults to:

```text
worlds/static_obstacles.world
```

The obstacles are static SDF boxes with collisions. That means they should be
visible to the laser and should not be pushed around by the robot.

## Run On The Real Robot

Start the real MIRTE robot bringup separately first. The robot must publish:

```text
/scan
/odom
/tf
/tf_static
```

Then run mapping without Gazebo:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false
```

In real-robot mode, `use_sim_time` automatically becomes `false`.

If the real robot uses different topic names, pass them as launch arguments:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  simulation:=false \
  scan_topic:=/your_scan_topic \
  odom_topic:=/your_odom_topic
```

## RViz

RViz starts by default and loads:

```text
rviz/getmap.rviz
```

Disable RViz:

```bash
ros2 launch simbiosys_mapping getmap.launch.py start_rviz:=false
```

Use a different RViz config:

```bash
ros2 launch simbiosys_mapping getmap.launch.py rviz_config:=/path/to/file.rviz
```

## Map Saving

`getmap_node` automatically saves the latest `/map` every 20 seconds after the
first map is received.

Default output:

```text
maps/mirte_map.yaml
maps/mirte_map.pgm
```

Change the output name or directory:

```bash
ros2 launch simbiosys_mapping getmap.launch.py \
  output_dir:=maps \
  map_name:=test_map
```

Change the auto-save interval:

```bash
ros2 launch simbiosys_mapping getmap.launch.py auto_save_period:=5.0
```

Disable periodic auto-save:

```bash
ros2 launch simbiosys_mapping getmap.launch.py auto_save_period:=0.0
```

Save manually while the node is running:

```bash
ros2 service call /getmap_node/save_map std_srvs/srv/Trigger "{}"
```

By default, `save_on_shutdown:=true` in the launch file, so the node also tries
to save the latest map when it exits.

## Launch Options

`getmap.launch.py` supports these arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `simulation` | `true` | `true` starts Gazebo; `false` uses real robot topics only. |
| `gazebo_gui` | `true` | Show or hide the Gazebo GUI when in simulation mode. |
| `world` | `static_obstacles.world` | Gazebo world used when `simulation:=true`. |
| `start_rviz` | `true` | Start RViz. |
| `rviz_config` | `rviz/getmap.rviz` | RViz config file. |
| `use_sim_time` | `auto` | `auto` follows `simulation`; can be forced to `true` or `false`. |
| `slam_params_file` | `config/slam_toolbox_mapping.yaml` | Parameter file for `slam_toolbox`. |
| `scan_topic` | `/scan` | Laser scan topic. |
| `odom_topic` | `/odom` | Odometry topic monitored by `getmap_node`. |
| `map_topic` | `/map` | Occupancy grid topic from `slam_toolbox`. |
| `output_dir` | `maps` | Directory where map files are saved. |
| `map_name` | `mirte_map` | Base name for `.yaml` and `.pgm` map files. |
| `auto_save_period` | `20.0` | Seconds between automatic saves. `0.0` disables periodic saving. |
| `save_on_shutdown` | `true` | Save once when `getmap_node` shuts down, if a map exists. |

Show available launch arguments:

```bash
ros2 launch simbiosys_mapping getmap.launch.py --show-args
```

## Useful Checks

Run these in another terminal after sourcing the workspace:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 run tf2_tools view_frames
```

Expected status in the `getmap_node` logs:

```text
Mapping inputs: scan=True, odom=True, map=True
```

If `/scan` is missing, the laser or simulation is not publishing.

If `/odom` is missing, robot odometry or the base controller is not publishing.

If `/scan` and `/odom` exist but `/map` is missing, check `slam_toolbox` and TF.

## Manual Teleop In Simulation

To drive while mapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Drive slowly while SLAM is running so the map can update cleanly.

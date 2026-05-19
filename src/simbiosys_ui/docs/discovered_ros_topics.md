# Discovered ROS Topics

Discovery date: 2026-05-17

## Workspace Search

Likely ROS 2 workspaces found:

- `/home/mark/ros2_ws`
- `/home/mark/src`
- `/home/mark/MDP`

The active project workspace for SimBioSys appears to be `/home/mark/MDP`.

## MIRTE Gazebo Setup

`src/mirte-gazebo` was not present in `/home/mark/MDP/src`, so it was cloned from:

```text
https://github.com/mirte-robot/mirte-gazebo
```

The `.repos` file was found at:

```text
src/mirte-gazebo/sources.repos
```

Dependencies were imported with:

```bash
vcs import src/ < src/mirte-gazebo/sources.repos
```

`rosdep install --from-paths src --ignore-src -r -y` could not complete because apt required sudo password interaction. It also reported unresolved rosdep keys for `ament_python` in the local SimBioSys Python packages.

`colcon build --symlink-install` could not complete because CMake was configured for Ninja, but Ninja/build compiler setup was unavailable:

```text
CMake was unable to find a build program corresponding to "Ninja".
CMAKE_C_COMPILER not set, after EnableLanguage
CMAKE_CXX_COMPILER not set, after EnableLanguage
```

## Launch Files Found

Likely MIRTE/Gazebo launch files:

- `src/mirte-gazebo/launch/gazebo_mirte_master_empty.launch.xml`
- `src/mirte-gazebo/launch/gazebo_mirte_master_navigation.launch.py`
- `src/mirte-gazebo/launch/gazebo_mirte_world_generated.launch.xml`
- `src/mirte-gazebo/launch/spawn_mirte_master.launch.xml`
- `src/simbiosys_bringup/launch/simulation_mirte_master.launch.py`

Other relevant launch files include:

- `src/simbiosys_bringup/launch/ui_only.launch.py`
- `src/simbiosys_bringup/launch/ui_system.launch.py`
- `src/simbiosys_bringup/launch/teleop_system.launch.py`
- `src/simbiosys_bringup/launch/mapping_system.launch.py`
- `src/mirte-ros-packages/mirte_bringup/launch/camera.launch.py`
- `src/mirte-ros-packages/mirte_teleop/launch/teleop_key.launch.py`
- `src/mirte-ros-packages/mirte_teleop/launch/teleop_joy.launch.py`

## Live MIRTE Gazebo Topics

Relevant topics observed in the running MIRTE Gazebo simulation:

- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)
- `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/depth/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)
- `/mirte_base_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/mirte_base_controller/odom` (`nav_msgs/msg/Odometry`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/groundtruth/odom` (`nav_msgs/msg/Odometry`)
- `/scan` (`sensor_msgs/msg/LaserScan`)

A manual forward command published to `/mirte_base_controller/cmd_vel_unstamped` moved the robot in simulation, so the UI publishes teleop Twist messages to that topic.

## UI Topic Choices

- teleop: `/mirte_base_controller/cmd_vel_unstamped`
- camera compressed: `/camera/image_raw/compressed`
- camera raw fallback: `/camera/image_raw`
- odom: `/mirte_base_controller/odom`
- map: `/map` remains configured for future use

No `/map` topic is available in the current simulation, so the UI defaults to the dummy greenhouse grid. Real `nav_msgs/msg/OccupancyGrid` map rendering is implemented and will activate when the configured map topic publishes.

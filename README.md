# SimBioSys: Autonomous Greenhouse Robot

ROS 2-based greenhouse robotics system for autonomous navigation, plant-health monitoring, operator interaction, and mobile manipulation on the MIRTE Master platform.

<p align="center">
  <img src="media/robot_demo.gif" alt="SimBioSys robot demo" width="650">
  <br>
  <em>SimBioSys performing autonomous greenhouse inspection and navigation.</em>
</p>

---

## Overview

**SimBioSys** is an autonomous greenhouse robot developed for the TU Delft RO47007 Multidisciplinary Project. The system is designed to help monitor tulip beds by combining autonomous navigation, plant perception, digital-twin-style visualization, and operator supervision.

The robot is built around the **MIRTE Master** platform and follows a reuse-first ROS 2 architecture: instead of replacing existing robotics stacks, the system coordinates established tools such as **SLAM Toolbox**, **Nav2**, **AMCL**, **MoveIt2**, OpenCV, and the MIRTE hardware interfaces.

The project goal was to build a practical mobile robot prototype able to:

* navigate through a greenhouse environment;
* build and use maps for autonomous operation;
* inspect plant beds and collect plant-health information;
* display robot and plant data through a user interface;
* support teleoperation and operator feedback;
* prepare the system architecture for flower picking and harvesting behavior.

---

## Demo Media

Recommended media to add to this section:

<table>
  <tr>
    <td align="center">
      <img src="media/slam_navigation.gif" alt="SLAM and navigation demo" width="360">
      <br>
      <em>SLAM and autonomous navigation</em>
    </td>
    <td align="center">
      <img src="media/ui_demo.png" alt="SimBioSys UI" width="360">
      <br>
      <em>Operator UI and plant-bed status</em>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="media/perception_boxes.png" alt="Perception bounding boxes" width="360">
      <br>
      <em>Flower / bug perception output</em>
    </td>
    <td align="center">
      <img src="media/flower_picking.gif" alt="Flower picking demo" width="360">
      <br>
      <em>Arm and gripper prototype behavior</em>
    </td>
  </tr>
</table>

---

## My Contribution: SLAM and Navigation

My main contribution focused on the **mapping, localization, and navigation stack** for the mobile base.

More specifically, I worked on:

* setting up and testing **SLAM Toolbox** for map creation;
* integrating saved maps with **Nav2 Map Server**;
* configuring localization with **AMCL**;
* validating the robot's odometry, LiDAR, and TF frames;
* testing autonomous navigation goals on the MIRTE platform;
* supporting the integration between mapping, navigation, and higher-level mission behavior;
* keeping the navigation stack compatible with both simulation and real-robot operation.

This work formed the basis for the robot's ability to move safely between greenhouse locations and approach plant beds for inspection.

---

## System Architecture

The repository contains a modular ROS 2 workspace organized around project-specific `simbiosys_*` packages.

| Package                | Purpose                                                                          |
| ---------------------- | -------------------------------------------------------------------------------- |
| `simbiosys_interfaces` | Custom messages, services, and actions                                           |
| `simbiosys_behavior`   | Mission coordinator, behavior requests, status topics, and Nav2 goal wrapper     |
| `simbiosys_mapping`    | SLAM Toolbox configuration, mapping helpers, and map-related utilities           |
| `simbiosys_base`       | Base-motion and path-planning utilities                                          |
| `simbiosys_perception` | Flower, plant, and bug perception components                                     |
| `simbiosys_arm`        | Arm and gripper wrappers for MIRTE manipulation                                  |
| `simbiosys_ui`         | Operator interface and dummy dashboard mode                                      |
| `simbiosys_bringup`    | Launch files and configuration for simulation, UI, mapping, and real-robot modes |

<p align="center">
  <img src="media/system_architecture.png" alt="SimBioSys system architecture" width="700">
  <br>
  <em>System-level ROS architecture and module interaction.</em>
</p>

---

## Core Technologies

* ROS 2
* Python
* Pixi
* colcon
* MIRTE Master
* SLAM Toolbox
* Nav2
* AMCL
* MoveIt2
* OpenCV / cv_bridge
* LiDAR, RGB-D camera, odometry, TF

---

## Setup

Install [Pixi](https://pixi.sh/latest/) if it is not already available.

Clone the repository:

```bash
git clone https://github.com/tommasocalzolari/simbiosys-greenhouse-robot.git
cd simbiosys-greenhouse-robot
```

Install the Pixi environment:

```bash
pixi install
```

Enter the Pixi shell:

```bash
pixi shell
```

Fetch the external MIRTE/ROS repositories listed in `repos.repos`:

```bash
pixi run vcs import --input repos.repos src
```

Ignore MIRTE packages that are not needed for this laptop-side workspace:

```bash
touch src/mirte-ros-packages/mirte_{bringup,telemetrix_cpp,teleop,test,zenoh_setup}/COLCON_IGNORE
```

Install dependencies and build:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Check that the custom interfaces are available:

```bash
ros2 interface show simbiosys_interfaces/srv/SendNamedArmPose
```

---

## Running the System

### Dummy UI Mode

Runs the terminal UI with fake dashboard data. Useful for development without the robot, Gazebo, Nav2, or cameras.

```bash
source install/setup.bash
ros2 launch simbiosys_bringup ui_system.launch.py
```

### Simulation Mode

Runs the MIRTE Master Gazebo simulation when the simulation packages are available.

```bash
source install/setup.bash
ros2 launch simbiosys_bringup simulation_mirte_master.launch.py
```

### Mapping Mode

Starts the mapping stack for building a map from LiDAR and odometry.

```bash
source install/setup.bash
ros2 launch simbiosys_bringup mapping_system.launch.py
```

### Teleoperation

Starts the teleoperation system.

```bash
source install/setup.bash
ros2 launch simbiosys_bringup teleop_system.launch.py
```

### Real Robot Laptop-Side Mode

Run the low-level MIRTE bringup on the robot first. Then, on the laptop:

```bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
ros2 launch simbiosys_bringup laptop_system.launch.py
```

Verify that the robot topics are visible:

```bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once
ros2 topic echo /mirte_base_controller/odom --once
ros2 topic echo /camera/color/image_raw --once
```

Check the main TF frames:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## Useful Launch Commands

```bash
# UI only
ros2 launch simbiosys_bringup ui_system.launch.py

# Teleoperation
ros2 launch simbiosys_bringup teleop_system.launch.py

# Mapping
ros2 launch simbiosys_bringup mapping_system.launch.py

# Real robot laptop-side system
ros2 launch simbiosys_bringup laptop_system.launch.py

# Arm wrapper test
ros2 launch simbiosys_bringup arm_test.launch.py
```

Safe named arm-pose service call:

```bash
ros2 service call /simbiosys/send_named_arm_pose simbiosys_interfaces/srv/SendNamedArmPose "{pose_name: home}"
```

---

## Report

A detailed project report is available here:

[Project report](docs/project_report.pdf)

---

## Suggested Media to Keep

For a concise portfolio README, the most useful media are:

1. **Robot demo GIF/video**
   Show the real MIRTE robot moving, navigating, or executing a task.

2. **SLAM/Nav2 map screenshot**
   Show the map, costmap, planned path, or RViz navigation output.

3. **UI screenshot**
   Show the digital-twin dashboard, plant-bed grid, status view, or operator controls.

4. **Perception output**
   Show flower, bug, or plant detections with bounding boxes.

5. **System architecture diagram**
   Use only one high-level architecture figure, preferably the ROS node architecture or functional-flow diagram.

Avoid adding too many report screenshots, tables, internal TODO lists, or long course-template sections. The README should show what the system does, how to run it, and what you contributed.

---

## Disclaimer

This repository is a public portfolio version of a TU Delft robotics project. Some hardware-specific files, private course infrastructure, datasets, or university-specific resources may not be included. The repository is intended to document the project architecture, implementation approach, and my contribution to the SLAM and navigation components of the system.

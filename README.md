# SimBioSys MIRTE ROS 2 Workspace

Official Group 06 repository for the SimBioSys ROS 2 workspace.

This repository keeps the Pixi/colcon MIRTE workspace setup from the course
template and adds our `simbiosys_*` packages on top. The project direction is
reuse-first: our code should wrap and coordinate existing MIRTE and ROS 2
packages instead of replacing them.

## Reuse-First MIRTE Strategy

We reuse:

- MIRTE Gazebo for simulation.
- MIRTE ros2_control topics/actions for base, arm, and gripper control.
- `slam_toolbox` for SLAM.
- `nav2_map_server` and `map_saver_cli` for map saving.
- `teleop_twist_keyboard` for manual driving.
- MIRTE MoveIt config when arm planning is needed.

Do not copy `mirte-documentation` into this repo. Use it only as documentation
and reference.

## Usage Modes

### 1. Simulation Mode

Runs the MIRTE Master Gazebo simulation when `mirte_gazebo` is installed.

```bash
pixi shell
colcon build
source install/setup.bash
ros2 launch simbiosys_bringup simulation_mirte_master.launch.py
```

### 2. Real Robot Laptop-Side Mode

Run low-level MIRTE bringup on the robot, then run our laptop-side shell.

```bash
source install/setup.bash
ros2 launch simbiosys_bringup laptop_system.launch.py
```

This does not launch robot hardware bringup.

### 3. Dummy Development Mode

Runs the terminal UI with fake dashboard data.

```bash
source install/setup.bash
ros2 launch simbiosys_bringup ui_system.launch.py
```

## Package Overview

| Package | Purpose |
| --- | --- |
| `simbiosys_interfaces` | Custom messages, services, and actions |
| `simbiosys_behavior` | Mode manager: `WAIT_FOR_OPERATOR`, `TELEOP`, `MAPPING`, `ARM_TEST`, `AUTONOMOUS_IDLE`, `ERROR` |
| `simbiosys_perception` | Lightweight flower detection placeholder and future perception hooks |
| `simbiosys_mapping` | `slam_toolbox` config and mapping topic status helper |
| `simbiosys_base` | High-level base/path placeholders |
| `simbiosys_arm` | Wrappers for MIRTE joint states, arm trajectory topic, and gripper action |
| `simbiosys_ui` | Terminal status UI and dummy dashboard data |
| `simbiosys_bringup` | Launch files and topic config for the three modes |

## Setup

If you do not have Pixi, install it from the
[Pixi installation guide](https://pixi.prefix.dev/latest/installation/).

Clone this repository:

```bash
git clone https://gitlab.tudelft.nl/cor/ro47007/2026/group_06/main-simbiosys.git $HOME/ro47007_mirte_ws
cd $HOME/ro47007_mirte_ws
```

Install the Pixi environment:

```bash
pixi install
```

Fetch the MIRTE/ROS package repositories listed in `repos.repos`:

```bash
pixi run vcs import --input repos.repos src
```

Ignore MIRTE packages that are not needed for this laptop-side workspace:

```bash
touch src/mirte-ros-packages/mirte_{bringup,telemetrix_cpp,teleop,test,zenoh_setup}/COLCON_IGNORE
```

Build:

```bash
pixi shell
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Clean build artifacts when needed:

```bash
pixi run ws-clean
pixi run clean-build
```

## Quickstart Commands

Teleop:

```bash
ros2 launch simbiosys_bringup teleop_system.launch.py
```

Mapping:

```bash
ros2 launch simbiosys_bringup mapping_system.launch.py
```

Arm wrapper test:

```bash
ros2 launch simbiosys_bringup arm_test.launch.py
```

MIRTE MoveIt, when installed:

```bash
ros2 launch mirte_moveit_config mirte_moveit.launch.py use_sim_time:=True
```

## Physical Robot

Connect to the MIRTE Master via Wi-Fi AP or Ethernet. On the robot, set the ROS
domain id and restart ROS:

```bash
export ROS_DOMAIN_ID=1
sudo service mirte-ros restart
```

In each laptop Pixi shell, use the same domain id and allow network discovery:

```bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
```

Verify that robot topics are visible:

```bash
ros2 topic list
```

## Topic Verification Checklist

```bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
ros2 action list
```

Important default interfaces:

| Purpose | Topic or Action |
| --- | --- |
| Base velocity | `/mirte_base_controller/cmd_vel_unstamped` |
| Arm trajectory | `/mirte_master_arm_controller/joint_trajectory` |
| Arm FollowJointTrajectory action | `/mirte_master_arm_controller/follow_joint_trajectory` |
| Gripper action | `/mirte_master_gripper_controller/gripper_cmd` |

Topic config lives in:

```text
src/simbiosys_bringup/config/real_robot_topics.yaml
src/simbiosys_bringup/config/simulation_topics.yaml
```

## Team Workflow

Branch from `main`, do not push directly to `main`, and use merge requests.

```bash
git switch main
git pull
git switch -c feature/<short-description>
```

Before opening a merge request, build locally and mention any manual robot or
simulation steps you used.

More practical notes are in [docs/](docs/README.md).

## Troubleshooting

- Python build problems: fully deactivate Anaconda or other virtual Python
  environment managers before setting up this repository.
- Shells other than Bash: source the matching setup file in `install`, such as
  `setup.zsh`.
- macOS blocked binaries: approve the Pixi-installed binary in System Settings,
  then rerun the command.
- Build problems: inspect the failing package output and use a clean build when
  cache state is suspicious.
- Pixi environment problems: run `pixi clean`, then `pixi install`.
- Missing packages in Pixi: add available ROS packages with commands such as
  `pixi add ros-humble-turtlesim`.
- Inconsistent group results: compare `git diff` and keep `pixi.lock` shared so
  everyone installs the same dependency versions.

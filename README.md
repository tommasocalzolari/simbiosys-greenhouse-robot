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
| `simbiosys_behavior` | Behavior coordinator, mode state, Nav2 goal wrapper, status topics, and harvest flag |
| `simbiosys_perception` | Lightweight flower detection placeholder and future perception hooks |
| `simbiosys_mapping` | `slam_toolbox` config and mapping topic status helper |
| `simbiosys_base` | High-level base/path placeholders |
| `simbiosys_arm` | Wrappers for MIRTE joint states, arm trajectory topic, and gripper action |
| `simbiosys_ui` | Terminal status UI and dummy dashboard data |
| `simbiosys_bringup` | Launch files and topic config for the three modes |

## Current Behavior System

The behavior layer is now a thin coordinator in `simbiosys_behavior`. It does
not replace SLAM, Nav2, perception, arm control, or the UI. It accepts behavior
requests, checks whether required topics/actions are alive, publishes status,
and delegates work to the existing robot stacks.

### General Topics And Services

| Purpose | Interface | Current owner | Status |
| --- | --- | --- | --- |
| Behavior command | `simbiosys/execute_behavior` action | `mission_manager_node` | Usable for mode changes and Nav2 navigation. Scan/harvest return `NOT_IMPLEMENTED` until wired. |
| General task status | `simbiosys/task_status` | `mission_manager_node` | Usable. Publishes current mission state and harvest flag. |
| Navigation status | `simbiosys/navigation_status` | `mission_manager_node` | Usable. Publishes phase, latest pose/path when available, target, progress, and error text. |
| Scan progress | `simbiosys/scan_progress` | `mission_manager_node` for now | Contract exists. Real scan executor is still TODO. |
| Harvest status | `simbiosys/harvest_status` | `mission_manager_node` for now | Contract exists. Physical harvest executor is still TODO. |
| Harvest flag set/get | `simbiosys/set_harvest_enabled`, `simbiosys/get_harvest_enabled` | `mission_manager_node` | Usable. Defaults to disabled. |
| Robot mode set | `simbiosys/set_robot_mode` | `mission_manager_node` | Usable compatibility service. |
| Plant health | `simbiosys/plant_health` | perception/analysis nodes | Usable typed topic; UI keeps legacy JSON fallback. |
| Bed observation | `simbiosys/bed_observation` | AprilTag perception | Usable typed topic. |
| Mapping status | `simbiosys/mapping_status` | mapping status node | Usable topic health helper. |

The real robot topic defaults remain:

| Capability | Default |
| --- | --- |
| Base velocity | `/mirte_base_controller/cmd_vel` |
| Odometry | `/mirte_base_controller/odom` |
| LiDAR | `/scan` |
| Map | `/map` |
| Color camera | `/camera/color/image_raw` |
| Depth camera | `/camera/depth/image_raw` |
| Depth point cloud | `/camera/depth/points` |
| Gripper camera | `/gripper_camera/image_raw` |
| Joint states | `/joint_states` |
| Arm trajectory | `/mirte_master_arm_controller/joint_trajectory` |
| Arm FollowJointTrajectory action | `/mirte_master_arm_controller/follow_joint_trajectory` |
| Gripper action | `/mirte_master_gripper_controller/gripper_cmd` |

Simulation keeps separate launch/config defaults, especially
`/mirte_base_controller/cmd_vel_unstamped` and `/odom`. Keep using launch args
or config files instead of hard-coding topic names in behavior code.

Real robot TF notes from the May 2026 smoke test:

- `/mirte_base_controller/odom` uses `frame_id: odom` and
  `child_frame_id: base_link`.
- `/scan` publishes `frame_id: laser`.
- The main camera topics use `camera_color_optical_frame` and
  `camera_depth_optical_frame`.
- Before SLAM, localization, Nav2, or perception work, verify that the sensor
  frames resolve from `base_link`:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
```

### Behavior Commands

| BehaviorType | What works now | What is still TODO |
| --- | --- | --- |
| `IDLE` | Cancels active Nav2 work when present, publishes zero velocity, enters idle. | Add arm-safe idle once arm behavior is integrated. |
| `TELEOP` | Sets mission state to teleop. | Safe teleop arbiter is still TODO; UI currently publishes velocity directly. |
| `MAP` | Sets mapping mode and checks scan/odom/map topics. | Finish-map command, bed annotation, cleanup, and metadata save are TODO. |
| `LOCALIZE` | Checks scan/odom/map and waits for `/amcl_pose`. | Initial-pose workflow remains external/RViz or launch driven. |
| `NAVIGATE` | Sends a Nav2 `NavigateToPose` goal from `target_pose`, with cancel and status. | Bed-ID approach pose from metadata is TODO. |
| `INSPECT_BED` | Accepts debug bed-side targets like `bed_1:a` and delegates to the dry-run-safe bed-side controller action. | Real metadata endpoint lookup, perception-driven servoing, and physical motion remain TODO. |
| `INSPECT_FLOWER` | Validates `target_id` and publishes scan status. | Single-position scan execution is TODO and currently returns `NOT_IMPLEMENTED`. |
| `HARVEST` | Enforces `harvest_enabled`; rejects while disabled. | Physical harvest is TODO and currently returns `NOT_IMPLEMENTED` even when enabled. |
| `ARM_TEST` | Preserves existing mode mapping. | Arm-specific execution remains in `simbiosys_arm`. |

Use `NAVIGATE` only after localization and Nav2 are launched. The behavior
manager does not start Nav2 for you; launch files still own system startup.

### Typed Metadata Contracts

These interfaces exist now and are intended for the next implementation slices:

- `BedRectangle.msg`: map-frame bed rectangle and AprilTag association.
- `ScanPosition.msg`: map-frame base pose. Reused for V1 bed-side `start` and
  `end` route endpoints such as `bed_1:a:start`.
- `MapMetadata.msg`: active map, cleaned map, beds, and scan positions.
- Metadata services: `SaveMapWithMetadata`, `LoadMapMetadata`,
  `UpsertBedRectangle`, `DeleteBedRectangle`, `SetScanPositions`, `CleanupMap`.

Important: the metadata services are contracts only right now. They are not yet
implemented by `simbiosys_mapping`. Teammates can implement those services
against YAML files under `maps/<map_id>/` without changing the behavior action
again.

### Team TODOs

Keep the next work in thin vertical slices. The behavior action and typed
metadata/status interfaces are already in place, so each area can integrate
against those contracts without changing the action payload immediately.

#### Arm Planning

- Validate named poses on the real robot: `scan`, `grab`, `remove`,
  `container_drop`, and `stow`.
- Keep the existing `SendNamedArmPose` service as the safe debug entry point.
- Report arm command success/failure clearly enough for behavior status and UI.
- Do non-cutting dry runs before connecting any physical harvest sequence.

#### Base Planning

- Verify `odom -> base_link`, `base_link -> laser`, and
  `base_link -> camera_link` before SLAM/Nav2 tests.
- Keep real robot defaults on `/mirte_base_controller/cmd_vel`,
  `/mirte_base_controller/odom`, and `/scan`; use launch args for simulation
  differences.
- Bring up SLAM/localization/Nav2 outside the behavior manager.
- Add bed approach pose computation from map metadata before full bed scanning.
- Do not add a custom global planner unless Nav2 is proven insufficient.

#### Perception

- Publish typed `simbiosys/plant_health` updates while keeping the legacy UI
  fallback until the UI is fully migrated.
- Use the real camera topics by default:
  `/camera/color/image_raw`, `/camera/depth/image_raw`, and
  `/gripper_camera/image_raw`.
- Include active bed, flower, and scan-position context in detection results.
- Report missed detections explicitly so scan behavior can retry or skip.

#### UI

- Connect UI commands to `simbiosys/execute_behavior`.
- Display `simbiosys/task_status`, `simbiosys/navigation_status`,
  `simbiosys/scan_progress`, and `simbiosys/harvest_status`.
- Add map annotation, scan-position editing, and cleanup controls around the
  typed metadata services.
- Keep dummy mode useful without a robot, Gazebo, Nav2, or camera data.

#### Behavior, Scheduling, And Interfaces

- Keep `mission_manager_node` as a thin coordinator; launch files own SLAM,
  localization, Nav2, perception, arm, gripper, and UI startup.
- Implement metadata read/write services in `simbiosys_mapping`.
- Add `ExecuteBehavior(NAVIGATE, target_id=<bed_id>)` by resolving bed approach
  poses from metadata.
- Grow the bed-side controller from dry-run scaffold to perception-driven
  distance/orientation/arm-height control.
- Keep `HARVEST` gated by `harvest_enabled` and returning `NOT_IMPLEMENTED`
  until scan results and arm/gripper poses are validated on the real robot.

Avoid adding a custom planner, automatic map cleanup, or a large new behavior
action payload until the current contracts are exercised by real callers.

See [Behavior API Examples](docs/behavior_api_examples.md) for command-line
smoke tests and [Behavior System](docs/behavior_sequences_simbiosys.md) for the
full implementation direction.

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

Enter the Pixi shell as its own command. Wait until the prompt changes, for
example to `(ro47007_mirte_ws)`, before running build or ROS commands:

```bash
pixi shell
```

Then run the workspace commands inside that Pixi shell:

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

Check that the generated SimBioSys interfaces are visible:

```bash
ros2 interface show simbiosys_interfaces/srv/SendNamedArmPose
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
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Run the commands above after entering `pixi shell`; do not paste `pixi shell`
and the later commands as one batch if your terminal does not wait for the new
Pixi shell to start.

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

In another Pixi shell, source the workspace and call the safe wrapper service:

```bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=0
ros2 service call /simbiosys/send_named_arm_pose simbiosys_interfaces/srv/SendNamedArmPose "{pose_name: home}"
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
ros2 topic echo /mirte_base_controller/odom --once
ros2 topic echo /camera/color/image_raw --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_tools view_frames
ros2 action list
```

Important default interfaces:

| Purpose | Topic or Action |
| --- | --- |
| Base velocity | `/mirte_base_controller/cmd_vel` |
| Odometry | `/mirte_base_controller/odom` |
| Main color image | `/camera/color/image_raw` |
| Depth image | `/camera/depth/image_raw` |
| Gripper camera image | `/gripper_camera/image_raw` |
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

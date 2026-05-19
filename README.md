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

## Behavior System Implementation Plan

The behavior system should use one shared execution framework in
`simbiosys_behavior`, with specialized behavior implementations underneath it.
The shared layer should own `simbiosys/execute_behavior` action intake,
behavior lifecycle state, cancellation, status/feedback publication, and
dispatch to mapping, Nav2, arm, perception, and gripper clients. Mapping,
navigation, scanning, and harvesting should not each invent separate command
systems.

Some behavior steps need special handling:

- Mapping is long-running and operator-driven, so it should be a workflow with
  manual finish, bed annotation, and cleanup confirmation steps.
- Autonomous movement should delegate planning, obstacle avoidance, path
  following, and replanning to Nav2.
- Scanning is a coordinated base-plus-arm loop over configured scan positions.
- Harvesting is gated by scan results and a system-level `harvest_enabled`
  flag; it should not run automatically unless explicitly enabled.

### Final Sanity Check And Recommendation

The first implementation should be intentionally boring. The real robot already
gives us enough to build useful behaviors, but only if we keep the first pass
close to the available interfaces:

- Base motion: `/mirte_base_controller/cmd_vel` on the real robot,
  `/mirte_base_controller/cmd_vel_unstamped` in simulation.
- Odometry: `/mirte_base_controller/odom` on the real robot, `/odom` in the
  current simulation config.
- LiDAR: `/scan`.
- Map: `/map`, produced by SLAM or Nav2 map server.
- Cameras: `/camera/color/image_raw`, `/camera/depth/image_raw`, and
  `/gripper_camera/image_raw` on the real robot.
- Arm and gripper: joint trajectory topic/action plus
  `/mirte_master_gripper_controller/gripper_cmd`.

That means the best V1 is a behavior coordinator, not a new autonomy stack. It
should check that required topics/actions are alive, call the existing ROS
systems, and publish understandable status. Avoid hiding too much behind clever
state-machine abstractions until mapping, localization, navigation, scanning,
and arm poses have each been proven independently.

Recommended V1 shape:

1. Keep launch files responsible for starting SLAM, localization, Nav2, camera,
   perception, arm, and UI nodes. The behavior manager should not try to spawn
   or kill complex launch systems in-process in the first version.
2. Add a small set of typed metadata and status interfaces first:
   `BedRectangle`, `ScanPosition`, `MapMetadata`, `ScanProgress`,
   `SetHarvestEnabled`, and map metadata save/load services.
3. Use the current `ExecuteBehavior` fields as far as possible before making
   the action very large: `target_pose` for generic navigation, `target_id` for
   map IDs, bed IDs, and flower IDs. Expand the action only when a real caller
   needs more typed fields.
4. Build `NAVIGATE` as a direct Nav2 `NavigateToPose` client. Do not implement
   custom path generation while Nav2 is available.
5. Build `INSPECT_BED` as sequential Nav2 goals plus arm named poses plus
   plant-health updates. Each scan position should be a full map-frame base
   pose so it can be tested one at a time.
6. Keep map cleanup semi-manual. Automatic cleanup is tempting, but operator
   confirmation is much easier to debug and safer for preserving walls.
7. Keep harvesting disabled by default until scanning reports stable flower
   IDs, heights, and flower-head centers. In early demos, scanning plus
   `ready_for_harvest` reporting is the robust milestone; physical harvest is
   the next milestone.

The current Nav2 config has `max_vel_y: 0.0`, so autonomous Nav2 movement should
be treated as forward/turning navigation even if manual teleop can publish
`linear.y`. Any behavior that depends on sideways base motion, especially
flower-head visual servoing during harvest, should first verify that the real
base controller and simulation both respond safely to lateral velocity. Until
that is confirmed, harvesting should prefer arm motion for vertical alignment
and conservative base yaw/forward corrections, with lateral servoing kept as a
separate tunable capability.

The simplest robust execution rule is: every behavior must have a dry-run or
debug mode that can be tested without the next subsystem. Navigation can run
without scanning. Scanning can run with manually supplied scan poses. Plant
health can update from fake perception. Harvest can run as an arm/gripper pose
sequence without cutting/removing anything. This keeps failures local and makes
field debugging much less painful.

### Behavior Type Mapping

The existing behavior action and enum are the integration point:

- `MAP`: full teleoperation-for-mapping workflow. Internally enables safe
  teleop, monitors SLAM, saves the cleaned map, and writes map metadata.
- `TELEOP`: manual driving/debug teleop without the full mapping lifecycle.
- `LOCALIZE`: start or check localization only.
- `INSPECT_BED`: scan every configured scan position for a bed.
- `INSPECT_FLOWER`: debug or targeted scan/rescan of one scan position or
  flower.
- `HARVEST`: explicit harvest target or scan-triggered harvest sequence, gated
  by `harvest_enabled`.
- `IDLE`: cancel active work and return the system to a safe idle mode.
- Add `NAVIGATE`: autonomous movement to a commanded map pose or computed bed
  approach pose. Navigation should not be overloaded onto `LOCALIZE`.

`ExecuteBehavior.action` should remain backward-compatible with its current
`behavior`, `target_id`, and `target_pose` fields. For V1, prefer using those
fields plus typed metadata services. Once real callers need richer command
payloads, expand the action with typed fields such as `map_id`, `bed_id`,
`flower_id`, `harvest_enabled`, `debug_override`, and optional scan-position
overrides. Feedback should include phase, current robot pose, target pose,
active path, active bed/flower, scan index/total, retry count, warnings, and
progress while keeping the existing `current_step` and `progress` fields useful
for older clients.

### New Typed Interfaces

Prefer typed interfaces over JSON/String topics, while keeping legacy fallbacks
where useful for the current UI and plant-health path.

Messages to add in `simbiosys_interfaces`:

- `BedRectangle.msg`: bed ID, map ID, map-frame center pose, length, width,
  yaw, and associated AprilTag IDs.
- `ScanPosition.msg`: scan-position ID, bed ID, flower ID, map-frame base pose,
  optional camera hint, order, and enabled flag.
- `MapMetadata.msg`: map ID, raw/cleaned map YAML paths, frame ID, bed
  rectangles, scan positions, and creation time.
- `NavigationStatus.msg`: phase, robot pose, target pose, current path,
  progress estimate, obstacle/replan status, and warning/error message.
- `ScanProgress.msg`: active bed, scan position, flower ID, scan index/total,
  detection status, retry count, latest plant-health summary, and message.
- `HarvestStatus.msg`: active flower, alignment status, harvest step, success,
  warning/error message, and timing information.

Services to add:

- `SaveMapWithMetadata.srv`
- `LoadMapMetadata.srv`
- `UpsertBedRectangle.srv`
- `DeleteBedRectangle.srv`
- `SetScanPositions.srv`
- `CleanupMap.srv`
- `SetHarvestEnabled.srv`
- `GetHarvestEnabled.srv`

The existing `MoveToTarget.action` can either become a thin Nav2 wrapper or be
left as a debug convenience. The behavior executor should call Nav2 actions
directly for production navigation.

### Map And Metadata Representation

Saved maps should remain normal Nav2 map artifacts, with SimBioSys metadata
stored next to them:

```text
maps/<map_id>/map_raw.yaml
maps/<map_id>/map_raw.pgm
maps/<map_id>/map_cleaned.yaml
maps/<map_id>/map_cleaned.pgm
maps/<map_id>/metadata.yaml
```

`metadata.yaml` should include schema version, map frame, resolution/origin
reference, raw and cleaned map file names, bed rectangles, AprilTag
associations, scan positions, creation time, and optional operator notes.

Bed rectangles should be stored as oriented 2D map-frame rectangles, not as UI
pixels: center position, yaw, length, and width. UI pixel coordinates may be
used temporarily while editing, but metadata should be map-coordinate data.

Scan positions should be persistent map-frame base poses associated with bed
and flower IDs. Normal scan positions are created during map annotation; debug
behavior calls may provide scan-position overrides without requiring a full map
creation flow.

Map cleanup should start as a semi-automatic workflow. The UI should let the
operator confirm keep/remove/free-space edits. `CleanupMap` then applies those
edits to the occupancy grid, preserving walls and permanent obstacles while
removing confirmed artifacts or temporary obstacles.

### Behavior Sequences

`MAP` teleoperation-for-mapping:

1. Accept `ExecuteBehavior(MAP, map_id)`.
2. Set mode/state to mapping.
3. Verify scan, odometry, and map topics through `MappingStatus`.
4. Enable safe UI teleop while SLAM is active.
5. Finish only when the UI sends an explicit finish command.
6. Stop the base and save the raw map.
7. Let the operator annotate bed rectangles and AprilTag associations.
8. Run map cleanup and save the cleaned map.
9. Write `metadata.yaml`.
10. Return to idle.

`NAVIGATE` autonomous movement:

1. Accept a generic map-frame pose or a bed ID.
2. Verify localization, cleaned map, map metadata, and Nav2 lifecycle state.
3. If a bed ID is provided, compute a bed approach pose from its rectangle.
4. Send a Nav2 `NavigateToPose` goal.
5. Publish current pose, target pose, active path, phase, replan status, and
   progress feedback.
6. Let Nav2 handle LiDAR obstacle avoidance and replanning.
7. On success, stop at the target and return idle or hand off to scanning.
8. On failure or cancel, cancel Nav2 and publish the reason.

Bed approach poses should be computed perpendicular to the bed's long side,
30 cm away from the rectangle, facing the bed center. If both sides are
possible, choose the side with a valid Nav2 plan, preferring the side closest to
the current robot pose or a configured aisle side.

`INSPECT_BED` scanning:

1. Validate the requested bed and its scan positions.
2. Move the arm to the scanning pose.
3. Navigate the base to each scan position while the arm stays in scan pose.
4. Wait for flower detection and plant analysis.
5. Retry each scan position up to the configured retry count if no flower is
   detected.
6. Publish typed `PlantHealth` for detected flowers.
7. Publish an explicit missed/unknown plant-health update when retries are
   exhausted.
8. If `harvest_enabled` is true and the plant is ready, invoke harvesting.
9. Stow or return the arm to idle at the end.

`INSPECT_FLOWER` should reuse the same scan implementation for one selected
scan position or flower ID.

`HARVEST`:

1. Confirm `harvest_enabled=true`, unless `debug_override=true`.
2. Require active scan context or an explicit flower target.
3. Use perception to track the flower-head bounding-box center.
4. Visual-servo by moving the base laterally and the arm vertically until the
   target is within configured pixel tolerances.
5. Abort safely if detection is lost or alignment times out.
6. Move to grabbing pose, close the gripper, remove the flower, move to the
   container/drop pose, open the gripper, and return to scanning pose.
7. Publish harvest status and update plant-health notes/status.

### Harvest Flag

`harvest_enabled` should be a system-level flag owned by `mission_manager_node`,
backed by a parameter and `SetHarvestEnabled`/`GetHarvestEnabled` services.
Default it to `false`. The UI should expose a clear toggle before autonomous
scanning starts. `INSPECT_BED` may trigger harvesting only when the flag is
true. Direct `HARVEST` requests should be rejected while disabled unless a debug
override is explicitly set.

### Cancellation And Failure Handling

All behaviors should support action cancellation:

- Cancel active Nav2 goals or publish zero base velocity.
- Stop scan loops at a safe boundary.
- Avoid unsafe abrupt arm/gripper interruption; move to a safe pose when
  possible.
- Return an action result with `success=false` and a clear message.

Useful failure classes are:

- `PRECONDITION_FAILED`: missing map, metadata, localization, Nav2, arm, or
  camera dependency.
- `PLANNING_FAILED`: Nav2 cannot find a path.
- `EXECUTION_FAILED`: controller/action failure.
- `PERCEPTION_TIMEOUT`: no flower or no flower-head target was detected.
- `HARVEST_DISABLED`: harvesting was requested while disabled.
- `SAFETY_ABORT`: visual servo timeout, unstable target, gripper fault, or arm
  fault.

### UI Changes

The UI should move from direct legacy commands toward behavior action requests
through its Python backend:

- Start and finish mapping.
- Annotate bed rectangles and associate AprilTags.
- Confirm map cleanup edits.
- Navigate to a clicked map pose.
- Navigate to a bed approach pose.
- Start bed scanning.
- Rescan one flower or scan position.
- Toggle `harvest_enabled`.
- Show behavior phase, progress, active bed/flower, retry count, warnings, and
  errors.
- Draw current robot pose, Nav2 path, target pose, bed rectangles, scan
  positions, and missed flowers.

Keep the existing `/plant_health` JSON fallback and dummy greenhouse dashboard
until typed end-to-end data is fully available. Keep `/ui/inspect_bed` as a
temporary bridge, but translate it internally to `ExecuteBehavior(INSPECT_BED)`.

### Package TODOs

`simbiosys_interfaces`:

- Add the typed map, navigation, scan, and harvest messages.
- Expand `ExecuteBehavior`.
- Add map metadata and harvest-enabled services.
- Add `NAVIGATE` to `BehaviorType`.

`simbiosys_behavior`:

- Replace the placeholder mission manager with behavior executors.
- Add a Nav2 action client.
- Add map, scan, and harvest orchestration.
- Add cancellation and failure classification.
- Publish typed status/progress topics.

`simbiosys_mapping`:

- Add metadata read/write helpers.
- Add map cleanup service/node.
- Extend map saving to preserve raw and cleaned map paths.
- Load map metadata alongside the active map.

`simbiosys_base`:

- Either remove placeholder path generation from the critical path or turn path
  execution into a thin Nav2 wrapper.
- Add a safe teleop arbiter if UI teleop should stop publishing directly to the
  base controller.

`simbiosys_arm`:

- Add named poses for scan, grab, remove, container drop, and stow.
- Add clearer arm command results.
- Add harvest helpers around MoveIt and the gripper service.

`simbiosys_perception`:

- Publish typed flower-head bounding box/center data for visual servoing.
- Consume active bed/flower/scan-position context in plant analysis.
- Publish missed detections explicitly.
- Keep typed `PlantHealth` and legacy JSON compatibility.

`simbiosys_ui`:

- Add behavior action/client endpoints.
- Add map annotation and cleanup UI.
- Draw paths, targets, bed rectangles, and scan positions.
- Add scan progress, missed-flower status, and harvest toggle.
- Keep dummy mode useful without robot, Gazebo, Nav2, or camera data.

`simbiosys_bringup`:

- Add behavior-system launch files and launch args for simulation/real robot,
  topic config, active map, metadata path, and harvest default.
- Preserve existing real robot vs simulation configurability through launch args
  and config files.

### Testing Plan

No hardware required:

- Interface generation/build.
- Metadata YAML read/write.
- Bed approach pose geometry.
- Scan-position ordering and retry state machine.
- Behavior goal validation and cancellation unit tests.
- UI dummy-mode behavior/status rendering.
- Plant-health updates and legacy JSON fallback.

Gazebo/Nav2 required:

- Navigation to a commanded pose.
- Bed approach pose planning.
- Replanning around simulated obstacles.
- Path feedback display.
- Mapping with `slam_toolbox` when scan, odometry, map, and TF are available.

MoveIt required:

- Arm scan, grab, stow, and container/drop poses.
- Harvest arm trajectory sequencing.
- Arm-motion failure and cancellation behavior.

Real robot required:

- Actual SLAM map quality.
- Real camera/depth flower detection and height estimates.
- AprilTag bed association reliability.
- Visual-servo tuning.
- Gripper harvest success.
- Safety limits for lateral base motion plus arm motion.
- Final harvest-enabled workflow validation.

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

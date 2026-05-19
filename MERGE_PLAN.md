# SimBioSys Branch Merge Plan

This plan is based on comparing `main` with:

- `origin/UI`
- `origin/arm_movement`
- `origin/feature/perception_olivier`
- `origin/planning`

The workspace currently has a clean `main` checkout. The packages of interest are under
`/home/sackmann/main-simbiosys/src`.

## What Each Branch Adds

### `origin/UI`

Scope: `simbiosys_ui`.

Adds a browser-based operator UI served by `ui_node.py`, plus documentation and a
JSON topic configuration file.

Important runtime contracts:

- Publishes teleop `geometry_msgs/Twist` to `/mirte_base_controller/cmd_vel_unstamped`.
- Publishes inspect-bed commands as `std_msgs/String` on `/ui/inspect_bed`.
- Subscribes camera streams:
  - `/camera/image_raw/compressed`
  - `/camera/image_raw`
- Subscribes mapping/localization:
  - `/map`
  - `/mirte_base_controller/odom`
  - `/amcl_pose`
- Subscribes plant-health JSON strings:
  - `/plant_health`
  - `/plant_health_report`
- Subscribes battery:
  - `/battery_state`
- Keeps backwards compatibility with existing `simbiosys/flower_data`
  (`simbiosys_interfaces/msg/FlowerData`) when available.

Compatibility notes:

- The UI currently uses `/plant_health` JSON as its preferred plant dashboard
  contract, while perception publishes `simbiosys/flower_data`.
- Its camera defaults are simulation-style `/camera/image_raw`, while current
  main/perception use `/camera/color/image_raw` and `/camera/depth/image_raw`.
- It adds `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, and `cv_bridge`
  dependencies to `simbiosys_ui`.

### `origin/arm_movement`

Scope: `simbiosys_arm`, small `simbiosys_bringup` import fix, and external
MIRTE-related submodules.

Adds a real `arm_motion_node.py` implementation around MoveIt:

- Provides action server `simbiosys/execute_arm_motion`
  (`simbiosys_interfaces/action/ExecuteArmMotion`).
- Sends goals to MoveIt `moveit_msgs/action/MoveGroup`, default `/move_action`.
- Supports motion types:
  - `pose`
  - `position`
- Subscribes `/joint_states` for a local numerical position IK fallback.
- Uses `moveit_msgs`, `shape_msgs`, `sensor_msgs`, `geometry_msgs`.

Updates `named_joint_pose_node.py`:

- Adds safer placeholder arm poses and aliases.
- Continues to provide service `simbiosys/send_named_arm_pose`
  (`simbiosys_interfaces/srv/SendNamedArmPose`).
- Publishes `trajectory_msgs/JointTrajectory` to
  `/mirte_master_arm_controller/joint_trajectory`.

Compatibility notes:

- The `ExecuteArmMotion.action` in `main` already matches what this branch uses:
  `target_pose`, `motion_type`, result success/message, feedback step/progress.
- Keep the `launch_xml` import fix in `simbiosys_bringup/launch_utils.py`; both
  `arm_movement` and `planning` want this.
- Decide whether to commit the external packages/submodules added under `src/`:
  `clearpath_mecanum_drive_controller`, `gazebo_grasp_fix`, `mirte-gazebo`,
  `mirte-ros-packages`. If those are dependencies, prefer tracking them through
  `repos.repos` rather than anonymous gitlink additions.

### `origin/feature/perception_olivier`

Scope: `simbiosys_perception`.

Adds camera-based perception:

- `apriltag_detection_node.py`
  - Subscribes `/camera/color/image_raw`.
  - Publishes JSON `std_msgs/String` on `/simbiosys/detected_tags`.
  - Publishes `std_msgs/Int32` current bed id on `/simbiosys/current_bed_id`.
- Replaces the placeholder `flower_detection_node.py` with OpenCV/cv_bridge
  Dahlia flower detection.
  - Subscribes `/camera/color/image_raw`.
  - Subscribes `/camera/depth/image_raw`.
  - Subscribes `/camera/depth/camera_info`.
  - Publishes `simbiosys/flower_data`
    (`simbiosys_interfaces/msg/FlowerData`).
  - Encodes height in `FlowerData.position.z` as centimeters.
- Updates `plant_analysis_node.py`.
  - Subscribes `simbiosys/flower_data`.
  - Publishes `simbiosys/plant_analysis`
    (`simbiosys_interfaces/msg/PlantAnalysis`).

Compatibility notes:

- This branch uses real robot style camera topics, unlike UI/planning defaults.
- AprilTag outputs should be converted from generic `String`/`Int32` to a shared
  interface if behavior/UI will use them for bed selection.
- `FlowerData` is already used by UI fallback, perception, and plant analysis.
  It can remain as the low-friction first integration contract.

### `origin/planning`

Scope: `simbiosys_mapping`, `simbiosys_bringup`, small arm/perception topic
changes, dependency updates.

Adds mapping/localization features:

- `getmap_node.py`
  - Subscribes scan, odom, map.
  - Saves map as `.pgm` plus `.yaml`.
  - Provides service `~/save_map` (`std_srvs/Trigger`).
- `initial_pose_node.py`
  - Publishes `/initialpose` for AMCL startup.
- `getmap.launch.py`
  - Optionally launches Gazebo.
  - Launches `slam_toolbox`, `getmap_node`, RViz.
- `localization.launch.py`
  - Launches Nav2 map server, AMCL, lifecycle manager, initial pose publisher,
    optionally Gazebo, and RViz.
- Adds AMCL config, richer slam_toolbox config, RViz configs, and a static
  obstacle world.

Compatibility notes:

- This branch changes many defaults from real robot topics to simulation topics:
  - `cmd_vel`: `/mirte_base_controller/cmd_vel_unstamped`
  - `odom`: `/odom`
  - camera: `/camera/image_raw`
- Do not blindly take these defaults into `main`. Keep launch arguments and
  profile-specific topic config so real robot and simulation both work.
- It removes the Pixi/Python workaround from `simbiosys_interfaces/CMakeLists.txt`;
  keep the workaround from `main` because interface generation depends on it in
  the Pixi/conda environment.
- It changes gripper limits/default effort. Prefer keeping main's clamped
  gripper limits and expose `max_effort` as a launch/config parameter.

## Existing Shared Interfaces

Already in `simbiosys_interfaces`:

- `msg/FlowerData`
  - Used by perception and UI fallback.
  - Good short-term contract for single flower detection.
- `msg/PlantAnalysis`
  - Used by `plant_analysis_node`.
  - Good short-term summary, but it does not identify bed/flower ids.
- `msg/TaskStatus`
  - Used by behavior manager and current simple UI.
  - Good for coarse mode/status display.
- `srv/SetRobotMode`
  - Used by behavior manager and current simple UI.
  - Should evolve carefully if "behavior types" become explicit commands.
- `srv/SendNamedArmPose`
  - Used by `named_joint_pose_node`.
  - Good for fixed inspection/stow/home pose commands.
- `action/ExecuteArmMotion`
  - Used by arm movement.
  - Keep as the low-level arm action.
- `action/MoveToTarget`
  - Reserved for navigation/base movement.
  - Not implemented yet by `simbiosys_base`.

## Interfaces To Add Or Evolve

### 1. Bed and tag detection

Replace `/simbiosys/current_bed_id` (`Int32`) and `/simbiosys/detected_tags`
(`String` JSON) with typed messages.

Recommended messages:

```text
# msg/DetectedTag.msg
int32 id
geometry_msgs/Point center_px
float32 area
float32 confidence
```

```text
# msg/BedObservation.msg
int32 bed_id
DetectedTag[] tags
bool visible
string message
```

Use:

- AprilTag node publishes `simbiosys/bed_observation`.
- Behavior subscribes to choose navigation/inspection behavior.
- UI subscribes to display current bed and tag confidence.

### 2. Plant health dashboard contract

UI currently wants `/plant_health` JSON with bed/flower ids. Perception currently
publishes `FlowerData` without ids.

Recommended message:

```text
# msg/PlantHealth.msg
string flower_id
string bed_id
float32 height_cm
string color
string health
string growth_stage
bool bug_detected
bool flower_detected
bool ready_for_harvest
float32 confidence
builtin_interfaces/Time last_scan_time
string notes
geometry_msgs/Point position
```

Use:

- Plant analysis publishes `simbiosys/plant_health`.
- UI subscribes to `simbiosys/plant_health` instead of `/plant_health` JSON.
- Keep `FlowerData` as the raw detector output feeding plant analysis.

### 3. Behavior types

Do not overload `SetRobotMode` for detailed behaviors. Keep it for coarse
operator modes, and add a behavior command/action for mission-level work.

Recommended enum-style message:

```text
# msg/BehaviorType.msg
uint8 IDLE=0
uint8 TELEOP=1
uint8 MAP=2
uint8 LOCALIZE=3
uint8 INSPECT_BED=4
uint8 INSPECT_FLOWER=5
uint8 HARVEST=6
uint8 ARM_TEST=7
uint8 type
```

Recommended action:

```text
# action/ExecuteBehavior.action
BehaviorType behavior
string target_id
geometry_msgs/Pose target_pose
---
bool success
string message
---
string current_step
float32 progress
```

Use:

- UI sends `ExecuteBehavior` goals such as `INSPECT_BED` with target `A`, `B`,
  or `C`.
- Behavior action server orchestrates base, perception, and arm actions.
- Behavior can call existing `MoveToTarget`, `ExecuteArmMotion`, and
  `SendNamedArmPose` internally.

### 4. Mapping status

The mapping nodes currently only log whether scan/odom/map are seen.

Recommended message:

```text
# msg/MappingStatus.msg
bool scan_seen
bool odom_seen
bool map_seen
bool localized
string active_map
string message
```

Use:

- `mapping_status_node` and/or `getmap_node` publishes
  `simbiosys/mapping_status`.
- UI and behavior use it instead of parsing logs.

## Merge Order

1. Create an integration branch from current `main`.

   ```bash
   git checkout main
   git pull --ff-only
   git checkout -b integration/all-branches
   ```

2. Merge `origin/feature/perception_olivier`.

   Reason: perception has the clearest typed-data flow and should become the
   source for plant-analysis/dashboard contracts.

   Resolve/verify:

   - Keep `/camera/color/image_raw`, `/camera/depth/image_raw`, and
     `/camera/depth/camera_info` as real robot defaults.
   - Add launch parameters/remaps later for simulation.
   - Keep `FlowerData` as raw detector output.

3. Merge `origin/arm_movement`.

   Reason: mostly independent and already aligned with existing interfaces.

   Resolve/verify:

   - Keep `moveit_msgs`, `shape_msgs`, and `geometry_msgs` dependencies.
   - Keep `ExecuteArmMotion.action` unchanged.
   - Keep/clean `launch_utils.py` import so `XMLLaunchDescriptionSource` comes
     from `launch_xml.launch_description_sources`.
   - Decide what to do with added external gitlinks before committing.

4. Merge `origin/UI`.

   Reason: UI needs to adapt to perception/mapping contracts chosen above.

   Resolve/verify:

   - Keep the UI topic config file.
   - Change UI defaults to use a shared real/sim topic profile instead of hard
     coded simulation topics.
   - Prefer typed `simbiosys/plant_health` once that interface exists.
   - Keep fallback subscription to `simbiosys/flower_data` during transition.

5. Merge `origin/planning` last.

   Reason: it touches shared launch/topic defaults and has the most runtime
   contract decisions.

   Resolve/verify:

   - Keep `main`'s `simbiosys_interfaces/CMakeLists.txt` Python/Pixi workaround.
   - Keep real robot defaults in `real_robot_topics.yaml`.
   - Add simulation defaults in `simulation_topics.yaml`.
   - Preserve launch arguments for `cmd_vel_topic`, `odom_topic`, `scan_topic`,
     `image_topic`, and `use_sim_time`; do not hard-code one environment.
   - Keep gripper clamping from `main`; expose `max_effort` as configurable.
   - Keep mapping/localization launch files, configs, RViz files, and nodes.
   - Avoid committing generated map files unless they are intentional test maps.

6. Add the new shared interfaces in a focused commit.

   Minimum useful set:

   - `DetectedTag.msg`
   - `BedObservation.msg`
   - `PlantHealth.msg`
   - `BehaviorType.msg`
   - `ExecuteBehavior.action`
   - optionally `MappingStatus.msg`

7. Adapt packages to the interfaces.

   - AprilTag node publishes `BedObservation`.
   - Plant analysis publishes `PlantHealth`.
   - UI subscribes to typed `PlantHealth` and `BedObservation`.
   - Behavior manager exposes `ExecuteBehavior` and keeps `SetRobotMode` for
     coarse operator mode.
   - Mapping status publishes `MappingStatus`.

8. Build and smoke test.

   ```bash
   pixi run build
   source install/setup.bash
   ros2 interface list | grep simbiosys_interfaces
   ros2 launch simbiosys_bringup laptop_system.launch.py
   ros2 run simbiosys_ui ui_node
   ```

## Expected Conflict Hotspots

- `pixi.lock`
  - `planning` changes dependency lock state. Regenerate after final
    `pixi.toml` is chosen rather than manually editing binary lock content.
- `pixi.toml`
  - Keep `colcon build --symlink-install` if desired.
  - Keep navigation/slam dependencies from `planning`.
  - Keep `ROS_LOCALHOST_ONLY=0` only if the team needs cross-machine ROS
    discovery by default.
- `src/simbiosys_bringup/simbiosys_bringup/launch_utils.py`
  - Both arm/planning touch the same XML import. Final version should import:
    `PythonLaunchDescriptionSource` from `launch.launch_description_sources` and
    `XMLLaunchDescriptionSource` from `launch_xml.launch_description_sources`.
- `src/simbiosys_perception/simbiosys_perception/flower_detection_node.py`
  - Take perception's implementation.
  - Do not take planning's one-line topic change as the final answer; solve this
    through launch/config profiles.
- `src/simbiosys_mapping/README.md`
  - Manual documentation conflict. Prefer planning's expanded docs, then add the
    real robot topic notes from `main`.
- `src/simbiosys_mapping/config/slam_toolbox_mapping.yaml`
  - Keep planning's richer config, but make `use_sim_time` overridden by launch.
- `src/simbiosys_interfaces/CMakeLists.txt`
  - Keep main's Python/Pixi workaround.

## Topic Policy

Use one canonical internal namespace for SimBioSys team topics:

- `simbiosys/task_status`
- `simbiosys/set_robot_mode`
- `simbiosys/execute_behavior`
- `simbiosys/execute_arm_motion`
- `simbiosys/send_named_arm_pose`
- `simbiosys/flower_data`
- `simbiosys/plant_analysis`
- `simbiosys/plant_health`
- `simbiosys/bed_observation`
- `simbiosys/mapping_status`

Keep hardware/simulation topics configurable:

- Real robot defaults:
  - cmd vel: `/mirte_base_controller/cmd_vel` or confirmed real unstamped topic
  - odom: `/mirte_base_controller/odom`
  - color image: `/camera/color/image_raw`
  - depth image: `/camera/depth/image_raw`
- Simulation defaults:
  - cmd vel: `/mirte_base_controller/cmd_vel_unstamped`
  - odom: `/odom`
  - image: `/camera/image_raw`

The final merged branch should not require editing Python source to switch
between real robot and simulation.


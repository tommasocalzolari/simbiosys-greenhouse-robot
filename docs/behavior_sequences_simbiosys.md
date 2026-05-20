# SimBioSys Behavior System

This document is the current implementation direction for the SimBioSys
behavior layer. It replaces the earlier planning artifact with a decision
document that is intended to guide implementation.

The main design principle is reuse-first: SimBioSys behavior code should
coordinate existing MIRTE, ROS 2, SLAM, Nav2, perception, arm, gripper, and UI
interfaces. It should not become a custom autonomy stack.

## Summary Decision

Use one shared behavior execution framework in `simbiosys_behavior`, with
specialized adapters underneath it.

The shared framework owns:

- `simbiosys/execute_behavior` action intake.
- Behavior lifecycle state.
- Cancellation and failure handling.
- Feedback/status publication.
- Dispatch to mapping, Nav2, perception, arm, gripper, and UI-facing helpers.

The behavior implementations remain specialized:

- Mapping is an operator-driven workflow around teleop, SLAM, map save, bed
  annotation, and map cleanup.
- Navigation delegates planning, obstacle avoidance, execution, and replanning
  to Nav2.
- Scanning is a sequential base-plus-arm-plus-perception loop over configured
  scan positions.
- Harvesting is a gated scan-dependent routine. It stays disabled by default
  until scanning and flower-head targeting are reliable.

## Robot Capability Assumptions

The first implementation should stay close to the real interfaces that are
available today.

Real robot defaults:

| Capability | Interface |
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

Simulation defaults:

| Capability | Interface |
| --- | --- |
| Base velocity | `/mirte_base_controller/cmd_vel_unstamped` |
| Odometry | `/odom` |
| LiDAR | `/scan` |
| Map | `/map` |
| Camera | `/camera/image_raw` |

Keep all differences configurable through launch arguments and config files.
The behavior layer should not hard-code real robot versus simulation topic
choices.

Real robot frame notes from the May 2026 compatibility smoke test:

- `/mirte_base_controller/odom` uses `frame_id: odom` and
  `child_frame_id: base_link`.
- `/scan` uses `frame_id: laser`.
- The main camera topics use `camera_color_optical_frame` and
  `camera_depth_optical_frame`.
- Before blaming behavior code for SLAM, Nav2, or perception failures, verify
  the TF chain with:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Recommended V1 Scope

The first version should be deliberately simple to execute and debug.

1. Launch files start SLAM, localization, Nav2, perception, arm, gripper, and
   UI nodes. The behavior manager checks availability and coordinates them; it
   does not spawn or kill complex launch systems in-process.
2. Keep `ExecuteBehavior.action` small for V1. Use its current `behavior`,
   `target_id`, and `target_pose` fields where possible, plus typed metadata
   services. Expand the action only when real callers need richer payloads.
3. Keep typed metadata and status interfaces stable before adding complex
   behavior logic.
4. Implement `NAVIGATE` as a direct Nav2 `NavigateToPose` action client.
5. Implement `INSPECT_BED` as sequential Nav2 goals plus arm named poses plus
   plant-health updates.
6. Keep map cleanup semi-manual with operator confirmation.
7. Keep physical harvesting disabled by default. In early demos, scanning and
   `ready_for_harvest` reporting are the robust milestone.

The current Nav2 configuration has `max_vel_y: 0.0`, so autonomous Nav2 motion
should be treated as forward/turning navigation. Manual teleop can publish
`linear.y`, but any behavior that depends on lateral base motion must first be
validated on both the real robot and simulation. Until then, harvest alignment
should prefer arm motion plus conservative yaw/forward corrections.

Every behavior should have a dry-run or debug path:

- Navigation can run without scanning.
- Scanning can run from manually supplied scan positions.
- Plant-health updates can be tested from fake perception.
- Harvesting can run as a non-cutting arm/gripper pose sequence.

## BehaviorType Mapping

Current enum values cover the V1 behavior names, including a dedicated
navigation behavior type for clarity.

| Behavior | Mapping |
| --- | --- |
| `IDLE` | Cancel or finish active work and return to safe idle. |
| `TELEOP` | Manual driving/debug teleop without the full mapping lifecycle. |
| `MAP` | Full teleoperation-for-mapping workflow. |
| `LOCALIZE` | Start or check localization only. |
| `INSPECT_BED` | Scan every configured scan position for a bed. |
| `INSPECT_FLOWER` | Debug or targeted scan/rescan of one scan position or flower. |
| `HARVEST` | Explicit harvest target or scan-triggered harvest, gated by `harvest_enabled`. |
| `ARM_TEST` | Existing arm debug behavior. |
| `NAVIGATE` | New value for autonomous movement to a map pose or bed approach pose. |

`LOCALIZE` should not be overloaded to mean autonomous movement. Localization is
a prerequisite for navigation, not the navigation behavior itself.

## ExecuteBehavior Contract

V1 should keep the existing action compatible:

```text
simbiosys_interfaces/BehaviorType behavior
string target_id
geometry_msgs/Pose target_pose
---
bool success
string message
---
string current_step
float32 progress
```

Use these fields in V1:

- `behavior`: selected behavior.
- `target_id`: map ID, bed ID, flower ID, or scan-position ID depending on the
  behavior.
- `target_pose`: generic map-frame pose for navigation, or an override pose for
  debug scanning.

Add richer fields later only when needed:

- `map_id`
- `bed_id`
- `flower_id`
- `harvest_enabled`
- `debug_override`
- `ScanPosition[] scan_positions_override`

Detailed status can also be published on separate typed topics instead of
making the action large immediately.

## New Interfaces To Add

Prefer typed interfaces over legacy JSON/String topics. Keep existing legacy
fallbacks until the UI and perception path are fully typed end-to-end.

Messages:

- `BedRectangle.msg`
  - `string bed_id`
  - `string map_id`
  - `geometry_msgs/Pose2D center`
  - `float32 length_m`
  - `float32 width_m`
  - `float32 yaw`
  - `int32[] apriltag_ids`
- `ScanPosition.msg`
  - `string scan_position_id`
  - `string bed_id`
  - `string flower_id`
  - `geometry_msgs/Pose2D base_pose`
  - `geometry_msgs/Pose camera_hint`
  - `uint32 order`
  - `bool enabled`
- `MapMetadata.msg`
  - `string map_id`
  - `string map_yaml_path`
  - `string cleaned_map_yaml_path`
  - `string frame_id`
  - `BedRectangle[] beds`
  - `ScanPosition[] scan_positions`
  - `builtin_interfaces/Time created_at`
- `NavigationStatus.msg`
  - phase, robot pose, target pose, current path, progress estimate,
    obstacle/replan status, and message.
- `ScanProgress.msg`
  - active bed, scan position, flower ID, scan index/total, detection status,
    retry count, latest plant-health summary, and message.
- `HarvestStatus.msg`
  - active flower, alignment status, current harvest step, success flag,
    warning/error message, and timing.

Services:

- `SaveMapWithMetadata.srv`
- `LoadMapMetadata.srv`
- `UpsertBedRectangle.srv`
- `DeleteBedRectangle.srv`
- `SetScanPositions.srv`
- `CleanupMap.srv`
- `SetHarvestEnabled.srv`
- `GetHarvestEnabled.srv`

## Map And Metadata Storage

Saved maps remain normal Nav2 map artifacts. SimBioSys metadata is stored next
to them.

```text
maps/<map_id>/map_raw.yaml
maps/<map_id>/map_raw.pgm
maps/<map_id>/map_cleaned.yaml
maps/<map_id>/map_cleaned.pgm
maps/<map_id>/metadata.yaml
```

`metadata.yaml` should contain:

- schema version
- map ID
- frame ID, usually `map`
- map resolution/origin reference
- raw and cleaned map file names
- bed rectangles
- AprilTag-to-bed associations
- scan positions
- creation time
- optional operator notes

Bed rectangles are stored as oriented 2D map-frame rectangles:

- center position in map coordinates
- yaw
- length
- width
- optional AprilTag IDs

Do not store bed rectangles as UI pixels except during temporary browser
editing. The persisted representation must be map-coordinate data.

Scan positions are persistent map-frame base poses associated with bed and
flower IDs. Normal scan positions are created during map annotation. Debug
flows may supply scan positions manually to test scanning without a full map
creation flow.

Map cleanup should be semi-manual in V1:

1. Operator marks free/occupied/keep regions in the UI or another simple tool.
2. `CleanupMap` applies those edits to the occupancy grid.
3. The cleaned map is saved as `map_cleaned.yaml` and `map_cleaned.pgm`.
4. Walls and permanent obstacles are preserved unless the operator explicitly
   changes them.

## Behavior 1: Teleoperation For Mapping

Intent: let the operator drive while SLAM is active, then annotate and clean the
map.

Behavior type: `MAP`.

Recommended V1 sequence:

1. UI sends `ExecuteBehavior(MAP, target_id=<map_id>)`.
2. Behavior manager sets state to mapping.
3. Behavior manager verifies `/scan`, odometry, `/map`, and expected TF are
   available through existing mapping status checks.
4. Operator drives using safe teleop.
5. SLAM runs from the mapping launch file.
6. UI sends explicit finish command.
7. Behavior manager stops the base.
8. Save raw map.
9. UI/operator confirms bed rectangles and AprilTag associations.
10. Save metadata.
11. Run semi-manual map cleanup.
12. Save cleaned map and final metadata.
13. Return to idle.

Implementation notes:

- Mapping should not run AMCL localization at the same time; SLAM owns
  `map -> odom` while mapping.
- The UI can keep direct teleop in the first version, but the preferred path is
  a safe teleop arbiter so behavior cancellation can reliably stop the base.
- Use `MAP` as the compound workflow. Do not require a separate external
  `TELEOP + MAP` command combination.

Failure and cancel behavior:

- Missing scan, odometry, map, or TF: reject or abort with
  `PRECONDITION_FAILED`.
- Cancel during mapping: stop base and leave any partial raw map unsaved unless
  the operator explicitly saves it.
- Map save failure: remain in mapping/annotation state and report the path or
  service failure.

## Behavior 2: Autonomous Move To Commanded Position

Intent: move to a map-frame pose or a computed bed approach pose.

Behavior type: add `NAVIGATE`.

Recommended V1 sequence:

1. UI sends `ExecuteBehavior(NAVIGATE, target_pose=<map pose>)` for generic
   navigation, or `ExecuteBehavior(NAVIGATE, target_id=<bed_id>)` for bed
   navigation.
2. Behavior manager verifies localization, cleaned map, map metadata, Nav2
   action availability, and current robot pose.
3. For a generic pose, send that pose to Nav2.
4. For a bed ID, load the bed rectangle and compute an approach pose.
5. Send a Nav2 `NavigateToPose` goal.
6. Forward phase, current pose, target pose, path, progress, and replan status
   to UI/status topics.
7. On success, return idle or hand off to scanning if requested by a higher
   workflow.
8. On failure or cancel, cancel Nav2 and publish the reason.

Bed approach pose:

- Compute the bed long side from the rectangle dimensions.
- Generate candidate approach poses perpendicular to the long side.
- Place the robot 0.30 m away from the rectangle edge.
- Orient the robot to face the bed center.
- Try the candidate side that has a valid Nav2 plan.
- Prefer the side closest to the current robot pose or a configured aisle side.

Implementation notes:

- Do not implement custom global path generation in `simbiosys_base` while Nav2
  is available.
- `simbiosys_base/path_execution_node.py` can become a thin Nav2 wrapper or be
  left out of the critical path.
- Nav2 handles LiDAR obstacle avoidance and replanning through its costmaps.
- The UI can draw `/plan`, AMCL pose, target pose, and behavior status.

Failure and cancel behavior:

- Missing localization or map metadata: `PRECONDITION_FAILED`.
- Nav2 cannot plan: `PLANNING_FAILED`.
- Nav2 controller aborts: `EXECUTION_FAILED`.
- Cancel: cancel active Nav2 goal and publish stop/idle status.

## Behavior 3: Scanning Mode

Intent: visit scan positions for a bed, collect plant data, and publish
plant-health updates.

Behavior types:

- `INSPECT_BED` for a full bed scan.
- `INSPECT_FLOWER` for one flower or scan-position debug/rescan.

Recommended V1 sequence:

1. UI sends `ExecuteBehavior(INSPECT_BED, target_id=<bed_id>)`.
2. Behavior manager loads scan positions for that bed.
3. Behavior manager moves the arm to a scanning named pose.
4. For each enabled scan position:
   1. Navigate the base to the scan-position pose with Nav2.
   2. Wait for camera/perception updates.
   3. Wait for `PlantHealth` or plant-analysis result.
   4. If no flower is detected, retry up to the configured retry count.
   5. If still missed, publish an explicit missed/unknown health update.
   6. If detected, publish/update typed plant health.
   7. If `harvest_enabled` is true and the flower is ready, invoke the harvest
      subroutine.
5. Return arm to scan, stow, or idle pose as configured.
6. Return behavior to idle.

Recommended defaults:

- `scan_retry_count`: 2 or 3.
- `scan_timeout_sec`: short enough for field debugging, for example 5-10 s.
- `scan_settle_sec`: small delay after navigation before reading perception.

Implementation notes:

- Movement between scan positions should be base-only in V1; the arm stays in
  scanning pose.
- Each scan position should be testable by itself.
- Plant IDs should be stable within metadata. Use `<bed_id>-<order>` or
  explicit operator-provided flower IDs until a stronger tracking system exists.
- Missed flowers should not abort the whole bed scan. Publish the miss and
  continue.
- Keep legacy plant-health JSON support in the UI while publishing typed
  `simbiosys/plant_health`.

Failure and cancel behavior:

- Missing scan positions: `PRECONDITION_FAILED`.
- Navigation failure for one scan position: mark that position failed and
  continue only if configured to do so; otherwise abort the bed scan.
- No flower after retries: publish `missed` and continue.
- Cancel: cancel active Nav2 goal, stop the scan loop, and return arm to safe
  pose when possible.

## Behavior 4: Harvesting Mode

Intent: harvest a flower that scanning marked as ready.

Behavior type: `HARVEST`, or an internal subroutine triggered from
`INSPECT_BED` when `harvest_enabled` is true.

Recommended V1 policy:

- Default `harvest_enabled` to false.
- The UI must explicitly enable harvesting before autonomous scanning can
  trigger it.
- Direct `HARVEST` requests should be rejected while disabled unless a debug
  override is explicitly set.
- Early demos should stop at scan results and `ready_for_harvest` reporting
  until flower-head targeting and arm/gripper poses are validated.

Recommended sequence once enabled:

1. Confirm `harvest_enabled=true`.
2. Confirm an active scan context or explicit flower target.
3. Confirm the latest plant-health result is ready for harvest.
4. Track flower-head bounding-box center.
5. Align using visual servoing within configured tolerances.
6. Move arm to grabbing pose.
7. Close gripper.
8. Remove flower.
9. Move to container/drop pose.
10. Open gripper.
11. Return to scanning pose.
12. Publish harvest status and update plant-health notes/status.

Important V1 limitation:

The current Nav2 config does not allow lateral autonomous velocity
(`max_vel_y: 0.0`). Harvest visual servoing should not depend on lateral base
motion until the real base controller and simulation have both been validated
for safe lateral commands. Prefer arm movement plus conservative yaw/forward
corrections first.

Failure and cancel behavior:

- Harvest disabled: `HARVEST_DISABLED`.
- No stable flower-head target: `PERCEPTION_TIMEOUT`.
- Visual servo timeout or unstable target: `SAFETY_ABORT`.
- Arm or gripper failure: `EXECUTION_FAILED`.
- Cancel: stop base commands, avoid unsafe gripper/arm interruption, and move
  to a safe pose when possible.

## Harvest-Enabled Flag

The harvest flag should be owned by `mission_manager_node`.

Implementation:

- Parameter: `harvest_enabled`, default `false`.
- Service: `SetHarvestEnabled`.
- Service: `GetHarvestEnabled`.
- UI toggle: visible before starting autonomous scan.
- Status: included in behavior status and UI status payload.

Rules:

- `INSPECT_BED` may trigger harvesting only when the flag is true.
- `HARVEST` rejects normal requests when the flag is false.
- Debug override must be explicit and visible in logs/status.

## Cancellation And Failure Model

All behavior implementations should share cancellation and failure categories.

Common cancel actions:

- Cancel active Nav2 goals.
- Publish zero base velocity when directly commanding teleop/base motion.
- Stop scan loops at a safe boundary.
- Return arm to a safe pose when possible.
- Publish final action result with `success=false` and a clear message.

Failure categories:

| Category | Meaning |
| --- | --- |
| `PRECONDITION_FAILED` | Missing map, metadata, localization, Nav2, arm, camera, or required topic/action. |
| `PLANNING_FAILED` | Nav2 cannot find a path. |
| `EXECUTION_FAILED` | Controller, action, arm, gripper, or service call failed. |
| `PERCEPTION_TIMEOUT` | No flower, plant result, or flower-head target was detected in time. |
| `HARVEST_DISABLED` | Harvest was requested while disabled. |
| `SAFETY_ABORT` | Servo timeout, unstable target, unsafe state, or conservative stop condition. |

## UI Changes

The UI should call behavior actions through its Python backend rather than
adding more legacy String topics.

Needed UI actions:

- Start mapping.
- Finish mapping.
- Annotate bed rectangles.
- Associate AprilTags with beds.
- Confirm map cleanup edits.
- Navigate to clicked map pose.
- Navigate to bed approach pose.
- Start bed scan.
- Rescan one flower or scan position.
- Enable/disable harvesting.
- Cancel active behavior.

Needed UI displays:

- Current behavior phase.
- Progress.
- Active bed and flower.
- Scan index and retry count.
- Detection status: `detected`, `retrying`, `missed`, `updated`.
- Current robot pose.
- Target pose.
- Current Nav2 path.
- Bed rectangles.
- Scan positions.
- Missed flowers.
- Warning or failure message.

Keep these fallbacks for now:

- `/plant_health` JSON for legacy dashboard updates.
- Dummy greenhouse dashboard data.
- `/ui/inspect_bed` as a temporary bridge to `ExecuteBehavior(INSPECT_BED)`.

## Package TODOs

### `simbiosys_interfaces`

- The V1 behavior, metadata, status, and harvest flag interfaces exist.
- Keep `ExecuteBehavior` compatible for V1.
- Add richer `ExecuteBehavior` fields only after real callers need them.
- Treat metadata services as contracts that mapping can implement without
  changing behavior action clients.

### `simbiosys_behavior`

- Keep `mission_manager_node` as a thin coordinator.
- Keep shared lifecycle, cancel, feedback, and failure handling centralized.
- Keep the Nav2 `NavigateToPose` client configurable by action name.
- Add metadata service clients.
- Add arm named-pose service client.
- Add gripper service/action client as needed.
- Add scan loop orchestration after single-position scan works.
- Keep harvest disabled by default until real scan and arm/gripper validation.

### `simbiosys_mapping`

- Add metadata read/write helper.
- Add `SaveMapWithMetadata`.
- Add `LoadMapMetadata`.
- Add map cleanup service/node.
- Preserve raw and cleaned map artifacts.
- Keep SLAM and localization launch behavior separate so `map -> odom` is not
  owned by SLAM and AMCL at the same time.

### `simbiosys_base`

- Do not build a competing global planner in V1.
- Either leave placeholder path generation out of the behavior path or convert
  path execution into a thin Nav2 wrapper.
- Add a safe teleop arbiter if behavior cancellation must reliably stop UI
  teleop.
- Verify real robot TF before mapping/navigation: `odom -> base_link`,
  `base_link -> laser`, and `base_link -> camera_link`.

### `simbiosys_arm`

- Add named poses: `scan`, `grab`, `remove`, `container_drop`, and `stow`.
- Keep the existing named-pose service for simple debugging.
- Add clearer result/status reporting for arm commands.
- Add harvest pose helpers only after the poses are validated.

### `simbiosys_perception`

- Publish typed plant-health updates.
- Keep legacy JSON compatibility through the UI.
- Add flower-head bounding-box/center output for future harvest servoing.
- Let plant analysis consume active bed/flower/scan-position context.
- Publish missed detections explicitly.

### `simbiosys_ui`

- Add behavior action endpoints in the Python backend.
- Add behavior status display.
- Add map annotation and cleanup UI.
- Draw robot pose, target pose, current path, bed rectangles, and scan
  positions.
- Add scan progress and missed-flower display.
- Add harvest-enabled toggle.
- Keep dummy mode useful without robot, Gazebo, Nav2, or camera data.

### `simbiosys_bringup`

- Add behavior-system launch files only as orchestration wrappers around
  existing package launches.
- Preserve launch arguments for simulation/real robot topic differences.
- Add launch/config parameters for active map, metadata path, and harvest
  default.

## Testing Plan

No hardware required:

- Interface generation and import checks.
- Metadata YAML read/write tests.
- Bed approach pose geometry tests.
- Scan-position ordering tests.
- Retry and missed-flower state-machine tests.
- Behavior goal validation tests.
- Cancel-path unit tests.
- UI dummy-mode rendering.
- Plant-health typed and legacy JSON update tests.

Gazebo/Nav2 required:

- Navigation to a clicked map pose.
- Navigation to computed bed approach pose.
- Replanning around simulated obstacles.
- UI path and target display.
- Mapping with SLAM when scan, odometry, map, and TF are available.

MoveIt required:

- Scan, grab, stow, and drop named poses.
- Harvest arm sequence dry run.
- Arm-motion failure and cancellation handling.

Real robot required:

- SLAM map quality.
- AMCL localization stability.
- Nav2 movement in the real greenhouse/test area.
- Bed approach pose distance and camera view.
- Real color/depth flower detection.
- AprilTag association reliability.
- Arm reach from scan positions.
- Gripper harvest sequence.
- Lateral base motion validation before using lateral visual servoing.
- Final harvest-enabled scan-to-harvest workflow.

## Implementation Order

Recommended order:

1. Interfaces and metadata.
2. Behavior status/failure/cancel skeleton.
3. Navigation behavior using Nav2.
4. Map metadata save/load and bed rectangles.
5. Scan-position metadata and single-position debug scan.
6. Full bed scan loop with retries and missed-flower updates.
7. UI behavior command/status integration.
8. Harvest-enabled flag.
9. Harvest dry run with arm/gripper poses.
10. Flower-head servoing and physical harvest only after real robot validation.

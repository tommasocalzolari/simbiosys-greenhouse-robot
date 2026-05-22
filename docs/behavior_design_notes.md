# Behavior Design Notes

This document is my working roadmap for the Behavior, Scheduling, and
Interfaces topic. It is meant to help me decide what I can build now, what I
should ask teammates about, and where I should wait for their packages to
stabilize before wiring real robot behavior together.

## Purpose

The behavior layer should stay a coordinator, not become a second mapping,
navigation, perception, arm, or UI stack. Its job is to accept
`ExecuteBehavior` goals, check preconditions, publish useful status, and
delegate work to the existing ROS 2 packages through typed interfaces.

The current interfaces are good enough for the next slices:

- `ExecuteBehavior` selects the behavior and carries either a `target_id` or a
  debug `target_pose`.
- Metadata messages describe maps, beds, and scan route endpoints.
- Status topics expose task, navigation, scan, and harvest state.
- Harvest flag services keep physical harvesting disabled by default.

The most important design choice is to keep package boundaries clean. Behavior
should call metadata services and action/service clients instead of reading
mapping YAML files directly or publishing low-level robot commands itself.

## Main Design Argument

Use a client-first behavior design:

1. Keep `mission_manager_node` as the scheduler and state owner.
2. Let launch files start SLAM, localization, Nav2, perception, arm, gripper,
   and UI nodes.
3. Use typed service/action clients for all cross-package work.
4. Fail clearly when required dependencies are missing.
5. Add behavior execution in thin slices that can be tested independently.

This keeps the behavior code debuggable. If Nav2 is not running, `NAVIGATE`
should say that Nav2 is unavailable. If bed-side route metadata is missing,
bed scanning should fail with a metadata precondition error. If harvesting is
disabled, `HARVEST` should reject the physical flow before any arm or gripper
action is attempted.

The behavior layer should not:

- parse map metadata YAML directly,
- compute its own global path instead of using Nav2,
- do image processing,
- send raw arm trajectories,
- hide the depth-camera servo controller inside `mission_manager_node`,
- hide missing teammate functionality behind fake success.

## What I Can Work On Now

### Metadata Clients And Preconditions

I can add behavior-side clients for metadata services once the mapping service
names and types are implemented. Until then, I can design the behavior code
around the existing service contracts and write tests with mocked clients.

Useful precondition checks:

- `target_id` is present when a behavior needs a bed or flower ID.
- `/map` and `/amcl_pose` have been seen before autonomous navigation.
- Nav2 `NavigateToPose` action server is available before sending a goal.
- Metadata service is available before resolving a bed ID.
- Arm, perception, and bed-side controller clients are available before scan
  execution.
- `harvest_enabled` is true before any harvest attempt.

### Navigation By Target Pose

The current `NAVIGATE` flow already sends a Nav2 goal from `target_pose`. I can
keep improving this path by making the failure messages and status publications
clearer, especially for missing map, localization, or Nav2.

This path is useful for testing before metadata is ready because the UI or CLI
can send a direct map pose.

### Navigation By Bed ID Or Bed Side

The next behavior slice should support:

```text
ExecuteBehavior(NAVIGATE, target_id=<bed_id>)
ExecuteBehavior(INSPECT_BED, target_id=<bed_id>:<side>)
```

The normal naming should be `bed_1`, `bed_2`, and so on. For side-specific
debugging, use side names `a` and `b`, for example `bed_1:a`. The side letters
avoid left/right ambiguity because left and right depend on the robot's travel
direction and which end of the bed is considered the start.

The behavior node should ask metadata for the active map, find the matching
bed, load the side's generated start/end route, navigate to the start pose with
Nav2, and then hand the local motion to a bed-side controller node.

### Bed-Side Scan Route Executor

The first scan implementation should be one side of one bed, not every bed in
memory. The mapping/UI side should generate one start and one end endpoint for
each long side of a bed after the bed rectangle is known. For now, reuse the
existing `ScanPosition` message by storing two entries per bed side:

- `bed_1:a:start`
- `bed_1:a:end`
- `bed_1:b:start`
- `bed_1:b:end`

These endpoints replace the older idea of many fixed scan positions. Flower
IDs are created at runtime when perception detects a flower and creates a
flower profile.

The debug behavior should run:

1. UI or CLI sends `ExecuteBehavior(INSPECT_BED, target_id=bed_1:a)`.
2. Behavior loads the `bed_1` side `a` start/end endpoints.
3. Nav2 drives to the start endpoint.
4. Arm moves to the named `scan` pose.
5. A bed-side controller node takes over local base and arm micro-control.
6. The controller moves in increments from start to end.
7. At each wait point, perception captures/analyzes an image.
8. If a flower is detected, perception creates a runtime flower profile.
9. If `harvest_enabled` is true and the flower is ready at the correct height,
   behavior schedules immediate harvest.
10. If the side ends without the minimum number of flowers, the robot retries
    once by scanning back along the same side.
11. After retry, behavior marks the bed side complete and schedules the next
    side, next bed, home, or idle depending on the active workflow.

The minimum flower count is a behavior parameter for now, scoped per bed side.
The UI may eventually provide the expected maximum/target flower count during
mapping, but behavior should keep a parameter fallback so debug scanning works
without complete UI metadata.

### Bed-Side Controller Node

Add a dedicated node in `simbiosys_behavior` for local bed-side scanning
control. `mission_manager_node` should schedule this node, not implement the
control loop itself.

The controller node should:

- subscribe to a perception message that estimates distance and relative
  rotation to the bed front plane from the depth camera,
- use that message to keep the base perpendicular to the bed long side,
- maintain the target distance from the side wall/front plane of the bed,
- move the base incrementally from the side start endpoint to the end endpoint,
- use flower bounding-box center information from perception to adjust arm
  height while the arm is in the scan pose,
- publish progress and failure state back to behavior.

The perception node should infer bed-relative distance and rotation from the
depth camera by seeing the bed legs and side wall/front plane. The controller
should not perform depth-image interpretation itself; it should consume typed
perception outputs.

### Status And Failure Polish

I can improve behavior value without moving the robot by making all failures
obvious to the UI and command line:

- `PRECONDITION_FAILED`: missing map, localization, metadata, target ID, arm,
  camera, bed-side controller, perception message, or action server.
- `PLANNING_FAILED`: Nav2 rejected or could not plan.
- `EXECUTION_FAILED`: Nav2, arm, perception, or scan execution failed after
  starting.
- `HARVEST_DISABLED`: harvest requested while disabled.
- `NOT_IMPLEMENTED`: accepted conceptually but no physical executor exists yet.

## What I Should Wait For

### Base Planning And Mapping

Wait for:

- reliable SLAM/localization/Nav2 launch flow,
- confirmation that `map -> odom -> base_link` works during localization,
- verified `base_link -> laser` and `base_link -> camera_link` transforms,
- real metadata service implementation,
- generated bed-side start/end endpoints for sides `a` and `b`.

I can still mock metadata clients and keep direct `target_pose` navigation
working while this is in progress.

### Perception

Wait for:

- typed `simbiosys/plant_health` updates,
- runtime flower profile output with generated flower IDs,
- depth-camera bed-relative distance and rotation output,
- flower bounding-box center output for arm-height micro-control,
- clear missed-detection reporting,
- a signal that perception has finished analyzing a detected flower.

Behavior should not guess whether a flower was missed. Perception should say
whether there was no detection, low confidence, bad depth, or a usable result.

### Arm Planning

Wait for:

- validated named poses: `scan`, `grab`, `remove`, `container_drop`, `stow`,
- a safe service/action response for named pose commands,
- confirmation that the `scan` pose is also a valid immediate harvest starting
  pose when the flower is at the correct height,
- dry-run harvest sequence validation,
- clear failure states when the arm cannot reach a pose.

Behavior can call a named-pose client later, but it should not send raw joint
trajectories or invent arm positions.

### UI

Wait for:

- final UI command flow into `simbiosys/execute_behavior`,
- which status topics the UI will display,
- metadata editing flow for bed rectangles and side start/end endpoints,
- how UI stores the expected maximum/target flowers per bed side,
- how the harvest-enabled toggle should be exposed to operators.

Behavior can keep publishing typed status now so the UI has stable data to
consume.

## Open Questions For Team

These decisions are now fixed for my design:

- Bed IDs use names like `bed_1`.
- Bed sides use `a` and `b`.
- Reuse `ScanPosition` for side start/end endpoints in V1.
- Flower IDs are generated at runtime when perception creates flower profiles.
- Debug scanning targets one side of one bed, for example `bed_1:a`.
- Retry threshold is a behavior parameter and applies per bed side.
- If `harvest_enabled` is true, `INSPECT_BED` may harvest immediately when
  perception marks a flower ready at the correct height.

Remaining questions for teammates:

1. What exact fields should the new bed-relative perception message contain?
   Minimum useful fields are distance error, yaw/rotation error, confidence,
   and whether the bed plane estimate is valid.
2. What exact fields should the flower-center perception message contain for
   arm-height control? Minimum useful fields are flower profile ID, bounding-box
   center, height estimate, readiness, and confidence.
3. Should the bed-side controller expose an action interface so
   `mission_manager_node` can start/cancel one side scan cleanly?
4. Which node should own publishing the runtime flower profile: perception
   directly, or behavior after receiving perception output?
5. What should the home/next-objective policy be after a debug side scan:
   return idle, return home, or stay at the side end pose?

## Recommended First Implementation Slice

The first implementation slice should avoid robot motion unless explicitly
tested with the team.

Recommended order:

1. Keep the current `ExecuteBehavior` action unchanged.
2. Add metadata service client structure in behavior, tested with mocks.
3. Add helper logic for resolving `target_id=bed_1:a` to side start/end
   endpoints once metadata is available.
4. Add clearer precondition failures for missing metadata, missing target IDs,
   missing localization, and missing action servers.
5. Add a bed-side controller node skeleton:
   - consume mocked bed-relative distance/rotation,
   - consume mocked flower-center perception,
   - expose start/cancel/status for one side route,
   - publish scan progress without commanding real motion at first.
6. Add an `INSPECT_BED` debug path for one side:
   - resolve side endpoints,
   - navigate to start,
   - request arm `scan` pose,
   - start the bed-side controller,
   - apply per-side minimum flower retry logic.
7. Keep `HARVEST` gated by `harvest_enabled`. When enabled, allow
   `INSPECT_BED` to schedule immediate harvest for flowers perception marks
   ready, but only after arm/gripper dry runs are validated.

## Test Plan

Documentation-only work does not require a robot or build. For future behavior
implementation, the tests should be layered:

### No Robot Required

- Unknown behavior type is rejected.
- Missing or malformed `target_id` fails for bed-side scans that require
  `bed_1:a` style identifiers.
- Missing metadata service returns `PRECONDITION_FAILED`.
- Missing Nav2 server returns `PRECONDITION_FAILED`.
- Harvest disabled returns `HARVEST_DISABLED`.
- Missing bed-side controller or perception stream returns
  `PRECONDITION_FAILED`.
- Side scan publishes `ScanProgress` with active bed, side/endpoints in the
  message text, retry count, and latest detected flower profile when available.
- Navigation failures publish `NavigationStatus` with `error=true`.
- Per-side minimum flower threshold triggers one retry when zero or too few
  flowers were detected.

### With Nav2 And Localization

- `NAVIGATE` to an explicit map pose succeeds or reports Nav2 failure.
- `INSPECT_BED` debug scan resolves `bed_1:a` metadata and navigates to the
  side start endpoint.
- Cancellation stops active Nav2 work and publishes zero velocity.

### With Real Robot

Before any autonomous behavior test:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link camera_link
```

Then test in this order:

1. Start `mission_manager_node` and verify services/actions.
2. Test harvest flag set/get.
3. Test behavior rejection and precondition failures.
4. Test Nav2 only after localization is known to work.
5. Test bed-side controller with fake perception messages before commanding
   real base or arm motion.
6. Test scan behavior only after base, arm, and perception teammates confirm
   their parts are ready.

No physical harvest test should run until the team has validated scan results,
named arm poses, gripper behavior, and an operator-controlled disable path.

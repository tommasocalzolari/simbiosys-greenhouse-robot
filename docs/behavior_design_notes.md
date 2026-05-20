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
- Metadata messages describe maps, beds, and scan positions.
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
should say that Nav2 is unavailable. If metadata is missing, bed navigation
should fail with a metadata precondition error. If harvesting is disabled,
`HARVEST` should reject the physical flow before any arm or gripper action is
attempted.

The behavior layer should not:

- parse map metadata YAML directly,
- compute its own global path instead of using Nav2,
- do image processing,
- send raw arm trajectories,
- hide missing teammate functionality behind fake success.

## What I Can Work On Now

### Metadata Clients And Preconditions

I can add behavior-side clients for metadata services once the mapping service
names and types are implemented. Until then, I can design the behavior code
around the existing service contracts and write tests with mocked clients.

Useful precondition checks:

- `target_id` is present when a behavior needs a bed, flower, or scan-position
  ID.
- `/map` and `/amcl_pose` have been seen before autonomous navigation.
- Nav2 `NavigateToPose` action server is available before sending a goal.
- Metadata service is available before resolving a bed ID.
- Arm/perception clients are available before scan execution.
- `harvest_enabled` is true before any harvest attempt.

### Navigation By Target Pose

The current `NAVIGATE` flow already sends a Nav2 goal from `target_pose`. I can
keep improving this path by making the failure messages and status publications
clearer, especially for missing map, localization, or Nav2.

This path is useful for testing before metadata is ready because the UI or CLI
can send a direct map pose.

### Navigation By Bed ID

The next behavior slice should support:

```text
ExecuteBehavior(NAVIGATE, target_id=<bed_id>)
```

The behavior node should ask metadata for the active map, find the matching
bed, compute or read an approach pose, and send that pose to Nav2.

Open design point: bed approach pose ownership should be discussed with Base
Planning and Mapping. Behavior can compute a simple pose from bed geometry, but
it may be cleaner for metadata/base planning to provide validated approach
poses directly.

### Single Scan-Position Executor

The first scan implementation should be one scan position, not a full-bed loop.
That gives the team one small integration target:

1. Navigate to the scan position.
2. Ask the arm to move to a named `scan` pose.
3. Wait for perception output for the active flower or scan-position ID.
4. Publish `ScanProgress`.
5. Return success, missed detection, or precondition failure.

This can be tested with mocked Nav2, arm, and perception before the real robot
flow is ready.

### Status And Failure Polish

I can improve behavior value without moving the robot by making all failures
obvious to the UI and command line:

- `PRECONDITION_FAILED`: missing map, localization, metadata, target ID, arm,
  camera, or action server.
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
- decision on whether bed approach poses are stored in metadata or computed by
  behavior.

I can still mock metadata clients and keep direct `target_pose` navigation
working while this is in progress.

### Perception

Wait for:

- typed `simbiosys/plant_health` updates,
- stable bed, flower, or scan-position IDs in perception output,
- clear missed-detection reporting,
- a decision on which camera is authoritative for each scan step.

Behavior should not guess whether a flower was missed. Perception should say
whether there was no detection, low confidence, bad depth, or a usable result.

### Arm Planning

Wait for:

- validated named poses: `scan`, `grab`, `remove`, `container_drop`, `stow`,
- a safe service/action response for named pose commands,
- dry-run harvest sequence validation,
- clear failure states when the arm cannot reach a pose.

Behavior can call a named-pose client later, but it should not send raw joint
trajectories or invent arm positions.

### UI

Wait for:

- final UI command flow into `simbiosys/execute_behavior`,
- which status topics the UI will display,
- metadata editing flow for beds and scan positions,
- how the harvest-enabled toggle should be exposed to operators.

Behavior can keep publishing typed status now so the UI has stable data to
consume.

## Open Questions For Team

These are the questions I should bring to teammates before implementing the
larger behavior slices:

1. Should bed approach poses be computed by behavior from `BedRectangle`, or
   stored as validated `ScanPosition`/approach metadata by Base Planning?
2. What should the stable scan-position ID format be?
3. Should UI request scanning by `bed_id`, `scan_position_id`, or `flower_id`?
4. What perception result should trigger a retry instead of a miss?
5. What arm response means the scan pose is safe and stable enough for image
   capture?
6. Should scan behavior wait for one fresh perception message, or call a future
   perception service/action to request analysis?
7. When harvest is eventually enabled, should it only run from an explicit
   `HARVEST` command, or can `INSPECT_BED` schedule harvest automatically for
   flowers marked ready?

My current recommendation is conservative:

- start with explicit commands,
- scan a single position first,
- require fresh typed perception output,
- keep harvest manual and disabled by default.

## Recommended First Implementation Slice

The first implementation slice should avoid robot motion unless explicitly
tested with the team.

Recommended order:

1. Keep the current `ExecuteBehavior` action unchanged.
2. Add metadata service client structure in behavior, tested with mocks.
3. Add helper logic for resolving `target_id=<bed_id>` to an approach pose once
   metadata is available.
4. Add clearer precondition failures for missing metadata, missing target IDs,
   missing localization, and missing action servers.
5. Add a single scan-position executor skeleton:
   - resolve scan position,
   - publish scan started,
   - call navigation helper,
   - call arm named-pose helper when available,
   - wait for perception result when available,
   - publish scan result.
6. Keep `HARVEST` gated by `harvest_enabled` and return `NOT_IMPLEMENTED` until
   real scan results and arm/gripper dry runs are validated.

## Test Plan

Documentation-only work does not require a robot or build. For future behavior
implementation, the tests should be layered:

### No Robot Required

- Unknown behavior type is rejected.
- Missing `target_id` fails for bed, flower, scan, and harvest behaviors that
  require one.
- Missing metadata service returns `PRECONDITION_FAILED`.
- Missing Nav2 server returns `PRECONDITION_FAILED`.
- Harvest disabled returns `HARVEST_DISABLED`.
- Scan placeholder publishes `ScanProgress` with useful message text.
- Navigation failures publish `NavigationStatus` with `error=true`.

### With Nav2 And Localization

- `NAVIGATE` to an explicit map pose succeeds or reports Nav2 failure.
- `NAVIGATE` by bed ID resolves metadata and sends the correct target pose.
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
5. Test scan behavior only after base, arm, and perception teammates confirm
   their parts are ready.

No physical harvest test should run until the team has validated scan results,
named arm poses, gripper behavior, and an operator-controlled disable path.

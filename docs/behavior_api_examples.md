# Behavior API Examples

These examples are small smoke tests for the current behavior interfaces. They
assume you are in the workspace and have sourced the install space.

```bash
pixi shell
source install/setup.bash
ros2 run simbiosys_behavior mission_manager_node
```

Run the calls below from another sourced shell.

## Harvest Flag

Check the current flag:

```bash
ros2 service call /simbiosys/get_harvest_enabled \
  simbiosys_interfaces/srv/GetHarvestEnabled "{}"
```

Enable harvesting:

```bash
ros2 service call /simbiosys/set_harvest_enabled \
  simbiosys_interfaces/srv/SetHarvestEnabled "{enabled: true}"
```

Disable harvesting again:

```bash
ros2 service call /simbiosys/set_harvest_enabled \
  simbiosys_interfaces/srv/SetHarvestEnabled "{enabled: false}"
```

## Behavior Status

Watch general behavior state:

```bash
ros2 topic echo /simbiosys/task_status
```

Watch navigation status:

```bash
ros2 topic echo /simbiosys/navigation_status
```

Watch scan and harvest status contracts:

```bash
ros2 topic echo /simbiosys/scan_progress
ros2 topic echo /simbiosys/harvest_status
```

## Navigate To A Pose

`NAVIGATE` is behavior type `8`. This requires localization and Nav2 to be
running. The behavior manager sends a Nav2 `NavigateToPose` goal using
`target_pose`.

```bash
ros2 action send_goal /simbiosys/execute_behavior \
  simbiosys_interfaces/action/ExecuteBehavior \
  "{
    behavior: {type: 8},
    target_id: '',
    target_pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }" \
  --feedback
```

## Mapping Mode Check

`MAP` is behavior type `2`. It currently sets mapping mode and checks whether
scan, odometry, and map topics have been seen. It does not yet save maps or
write metadata.

```bash
ros2 action send_goal /simbiosys/execute_behavior \
  simbiosys_interfaces/action/ExecuteBehavior \
  "{behavior: {type: 2}, target_id: test_map, target_pose: {orientation: {w: 1.0}}}" \
  --feedback
```

## Bed-Side Scan Debug Path

`INSPECT_BED` is behavior type `4`. The current debug implementation expects a
bed-side target in the form `<bed_id>:<side>`, where side is `a` or `b`, for
example `bed_1:a`. With the default dry-run settings, it verifies the behavior
and bed-side controller action plumbing without commanding robot motion.

```bash
ros2 action send_goal /simbiosys/execute_behavior \
  simbiosys_interfaces/action/ExecuteBehavior \
  "{behavior: {type: 4}, target_id: bed_1:a, target_pose: {orientation: {w: 1.0}}}" \
  --feedback
```

`INSPECT_FLOWER` is behavior type `5` and has the same placeholder status for a
single flower or scan position.

```bash
ros2 action send_goal /simbiosys/execute_behavior \
  simbiosys_interfaces/action/ExecuteBehavior \
  "{behavior: {type: 5}, target_id: A1, target_pose: {orientation: {w: 1.0}}}" \
  --feedback
```

## Harvest Placeholder

`HARVEST` is behavior type `6`. It rejects requests while harvesting is
disabled. When enabled, it still returns `NOT_IMPLEMENTED` until the physical
arm/gripper sequence is validated.

```bash
ros2 action send_goal /simbiosys/execute_behavior \
  simbiosys_interfaces/action/ExecuteBehavior \
  "{behavior: {type: 6}, target_id: A1, target_pose: {orientation: {w: 1.0}}}" \
  --feedback
```

# Suggested Non-UI Changes

These backend additions would let the UI move from unavailable states to real
commands/data. They are not implemented in the UI package.

## Safety / Manual Control

- Add a robot-side command arbiter or service for UI take-control/manual-control.
- Suggested service: `/ui/take_control`, `std_srvs/srv/Trigger`.
- Suggested release service: `/ui/release_control`, `std_srvs/srv/Trigger`.
- Behavior: cancel autonomous navigation, stop base motion, and grant/release
  teleop authority to the UI.

## Full Stop

- Add a backend full-stop interface for all robot/autonomous motion.
- Suggested service: `/ui/stop_all_motion`, `std_srvs/srv/Trigger`.
- Behavior: cancel active navigation/actions, publish or command zero base
  velocity, stop/hold arm motion, and put command arbitration into a stopped
  state.
- Do not automatically resume motion on START; require explicit operator action.

## Mapping Review

- Add `/mapping/start`, `std_srvs/srv/Trigger`.
- Add `/mapping/done`, `std_srvs/srv/Trigger`.
- Add `/mapping/artifact_candidates` with candidate ID, geometry, source, and
  classification fields.
- Add `/mapping/classify_artifact` for `wall`, `bed`, `obstacle`, and
  `false_scan`.
- Add `/mapping/save_safe_map` and `/mapping/safe_map` for reviewed safe maps.

## Bed Telemetry

- Add real bed-level telemetry for CO2, humidity, and bugs detected.
- Suggested topic: `/simbiosys/bed_telemetry`.
- Message should include stable bed ID, CO2, humidity, bugs detected yes/no,
  timestamp, and source.

## Arm Camera

- Publish a compressed arm/gripper camera topic if available, for example
  `/gripper_camera/image_raw/compressed`.
- Keep raw `/gripper_camera/image_raw` available as fallback.

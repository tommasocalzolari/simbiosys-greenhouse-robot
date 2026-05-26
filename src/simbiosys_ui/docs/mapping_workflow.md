# Mapping Workflow

## Real Discovered Interfaces

- `/map` (`nav_msgs/msg/OccupancyGrid`) is published by `slam_toolbox` during
  mapping and by map server/localization flows when a saved map is loaded.
- `simbiosys/mapping_status` (`simbiosys_interfaces/msg/MappingStatus`) is
  published by `simbiosys_mapping/mapping_status_node.py`.
- `/getmap_node/save_map` (`std_srvs/srv/Trigger`) is provided by
  `simbiosys_mapping/getmap_node.py` and saves the latest raw occupancy grid to
  `.yaml`/`.pgm` files.

Real mapping package interfaces not found:

- `/mapping/start`
- `/mapping/done`
- `/mapping/save_safe_map`
- `/mapping/artifact_candidates`
- `/mapping/classify_artifact`
- `/mapping/safe_map`
- false-scan removal service/action/topic

Real mapping artifact candidate interface not implemented yet.

## Current UI Behavior

- The live map panel subscribes to the configured `liveMap` topic.
- The UI renders real `nav_msgs/msg/OccupancyGrid` messages with occupied,
  free, and unknown cells.
- Rendering and status payload map transfer are throttled by `mapUpdatePeriodSec`, currently 10 seconds.
- The Teleop / Camera SLAM map draws the robot marker from real odometry or
  AMCL pose when available.
- The Dashboard map allows selecting a real map position but does not draw the
  robot marker.
- **Start Mapping** is enabled only when the configured start service is
  available.
- **Done Mapping** is disabled until a real map message has arrived.
- Pressing **Done Mapping** freezes the latest real received map in the browser
  for review.
- Pressing **Done Mapping** also freezes the latest real artifact candidates
  received from `/mapping/artifact_candidates`.
- If no artifact candidates have been received, review mode shows `No artifact
  candidates received` and does not create placeholder candidates.
- The candidate list is shown from received candidates even when a candidate's
  geometry cannot be rendered.
- Received candidates can be selected and locally classified for review as
  `wall`, `bed`, `obstacle`, or `false_scan`.
- `bed` review previews are rendered as rectangles. If a bed candidate arrives
  as a polygon, the UI previews its bounding rectangle.
- `false_scan` candidates remain visible in the list, but classified
  `false_scan` candidates are hidden from the map preview.
- **Retry** discards local review classifications and returns to the latest real
  live map.
- **Save Safe Map** is enabled only when a reviewed map exists, artifact
  candidates were received, at least one local classification was selected, and
  the configured save backend is available.
- The panel shows diagnostics for map topic state, last map timestamp, candidate
  count, Start Mapping backend, Save Safe Map backend, selected candidate ID,
  and review mode state.

## Missing Backend Contracts

- No real start-mapping service/topic/action was found.
- No real done/finalize-mapping service/topic/action was found.
- No real artifact classification backend was found.
- No real false-scan removal backend was found.
- No real reviewed safe-map save/publish backend was found.

Because real backend command contracts are missing, the UI shows disabled or
unavailable states instead of creating placeholder maps, candidates, backend
classifications, or save results. The separate test publisher provides
test-only Trigger services for `/mapping/start`, `/mapping/done`, and
`/mapping/save_safe_map` because those names match the UI configuration.

## Artifact Candidate Schema

The intended integration topic is `/mapping/artifact_candidates`
(`std_msgs/msg/String` JSON). Until the real mapping package implements it, the
test publisher is the only publisher in this repository. It sends:

- top-level `timestamp`
- top-level `frame_id`
- top-level `candidates` array
- candidate `id`, `kind`, `suggested_class`, `source`, `confidence`
- nested `geometry.type`
- nested rectangle geometry as `geometry.x`, `geometry.y`, `geometry.width`,
  and `geometry.height`
- nested polygon geometry as `geometry.points`

The UI also accepts older flat candidate fields for compatibility:

- `geometry_type`
- `points`
- `pose`
- `size`
- `suggested_class`

or nested fields:

- `kind`
- `geometry.type`
- `geometry.points`
- `geometry.pose`
- `geometry.size`
- `geometry.x`, `geometry.y`, `geometry.width`, `geometry.height`
- `geometry.radius`
- `geometry.center`
- `geometry.start`
- `geometry.end`

Candidates are normalized to one internal shape containing `id`, `classHint`,
`geometryType`, geometry fields, and `raw`. Unknown or unexpected fields do not
crash the UI; the candidate remains listed and a render/parse warning is shown
when needed.

## Verification Commands

```bash
ros2 topic list -t | grep -E "mapping|artifact|candidate|map|bed|environment"
ros2 service list -t | grep -E "mapping|map|safe|start|done|save"
ros2 topic echo /mapping/artifact_candidates --once --field data
ros2 topic echo /map --once
ros2 service call /mapping/start std_srvs/srv/Trigger {}
ros2 service call /mapping/done std_srvs/srv/Trigger {}
ros2 service call /mapping/save_safe_map std_srvs/srv/Trigger {}
```

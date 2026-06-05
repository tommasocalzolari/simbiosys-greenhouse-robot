# Mapping Schema Comparison

This document compares the real mapping package contracts discovered in this
repository with the standalone UI test-data publisher.

## Repository Search Summary

Commands used from `/home/mark/MDP`:

```bash
find src \( -iname "*map*" -o -iname "*slam*" \) -print
rg -n "artifact|candidate|mapping/start|mapping/done|save_safe_map|safe_map|classify|false_scan|/getmap_node/save_map|save_map" src --glob '!build/**' --glob '!install/**' --glob '!log/**'
rg -n "/mapping|/map|artifact_candidates|safe_map" src tools/test_publishers
```

Mapping/SLAM-related packages and files found:

- `src/simbiosys_mapping`
- `src/simbiosys_mapping/launch/getmap.launch.py`
- `src/simbiosys_mapping/config/slam_toolbox_mapping.yaml`
- `src/simbiosys_mapping/simbiosys_mapping/getmap_node.py`
- `src/simbiosys_mapping/simbiosys_mapping/mapping_status_node.py`
- `src/simbiosys_bringup/launch/mapping_system.launch.py`
- `src/simbiosys_interfaces/msg/MappingStatus.msg`
- `src/simbiosys_interfaces/msg/MapMetadata.msg`
- `src/simbiosys_interfaces/srv/SaveMapWithMetadata.srv`
- `src/simbiosys_interfaces/srv/CleanupMap.srv`
- `src/simbiosys_interfaces/srv/LoadMapMetadata.srv`

Real mapping package interfaces found:

| Purpose | Name | Type | Owner | Notes |
| --- | --- | --- | --- | --- |
| Live map | `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` during mapping, Nav2 map server during localization | Used by UI map display. |
| Mapping status | `simbiosys/mapping_status` | `simbiosys_interfaces/msg/MappingStatus` | `simbiosys_mapping/mapping_status_node.py` | Publishes `scan_seen`, `odom_seen`, `map_seen`, `localized`, `active_map`, `message`. |
| Raw map save | `/getmap_node/save_map` | `std_srvs/srv/Trigger` | `simbiosys_mapping/getmap_node.py` | Saves latest raw occupancy grid to `.yaml` and `.pgm`; not a reviewed safe-map backend. |

Real mapping package interfaces not found:

- `/mapping/start`
- `/mapping/done`
- `/mapping/save_safe_map`
- `/mapping/artifact_candidates`
- `/mapping/classify_artifact`
- `/mapping/safe_map`
- false-scan removal service/action/topic

Real mapping artifact candidate interface not implemented yet.

## Artifact Candidate Schema

Because the real mapping package does not currently publish artifact
candidates, the test publisher follows the documented intended UI integration
schema.

Real mapping package output topic name:

- Not implemented.
- Intended topic: `/mapping/artifact_candidates`.

Real message type:

- Not implemented.
- Intended current integration type: `std_msgs/msg/String` containing JSON.

Real JSON/message schema:

- Not implemented.

Test publisher topic name:

- `/mapping/artifact_candidates`

Test publisher type:

- `std_msgs/msg/String`

Test publisher schema:

```json
{
  "timestamp": "2026-05-22T00:00:00+00:00",
  "frame_id": "map",
  "candidates": [
    {
      "id": "artifact_2",
      "candidate_type": "unclassified",
      "source": "slam",
      "confidence": 0.9,
      "geometry": {
        "type": "rectangle",
        "pose": {"x": 1.0, "y": 1.2, "theta": 0.0},
        "size": {"width": 1.4, "height": 4.8}
      }
    },
    {
      "id": "artifact_4",
      "candidate_type": "unclassified",
      "source": "slam",
      "confidence": 0.82,
      "geometry": {
        "type": "polygon",
        "points": [
          {"x": 3.0, "y": 2.0},
          {"x": 3.5, "y": 2.1},
          {"x": 3.4, "y": 2.6}
        ]
      }
    }
  ]
}
```

UI expected schema:

- JSON object with `candidates` array, or a JSON array of candidate objects.
- Candidate fields: `id`, neutral `candidate_type`, `source`, and optional
  `confidence`.
- Geometry fields may be nested under `geometry`.
- Supported geometry fields: `geometry.type`, `geometry.points`,
  `geometry.pose`, `geometry.size`, `geometry.x`, `geometry.y`,
  `geometry.width`, `geometry.height`, `geometry.radius`, `geometry.center`,
  `geometry.start`, `geometry.end`, and `geometry.segments`.
- Older flat geometry fields are still accepted for compatibility:
  `geometry_type`, `points`, `pose`, and `size`.
- `suggested_class`, `class`, and semantic `kind` fields must not be used by
  the test publisher.

Differences/mismatches:

- No real mapping artifact-candidate schema exists to compare against.
- The test publisher matches the intended documented UI contract with numbered
  unclassified artifacts.
- The UI parser supports a small compatibility superset, but the test publisher
  uses the nested intended schema only.
- The UI does not infer classification from ID, geometry, `candidate_type`, or
  old suggested-class fields. Classification is manual.

Save Safe Map behavior:

- Manually classified `wall`, `bed`, and `obstacle` artifacts are kept in the
  saved object list.
- Manually classified `false_scan` artifacts are excluded from the safe map.
- Artifacts still `unclassified` at save time are treated as `false_scan` and
  excluded from the safe map.
- The UI save review result has this shape:

```json
{
  "objects": [
    {"id": "artifact_1", "class": "wall"}
  ],
  "removed_false_scans": [
    "artifact_4",
    "artifact_5"
  ]
}
```

Required changes:

- None to the real mapping package.
- Keep test-only artificial artifact data in `tools/test_publishers/`.
- When the real mapping package implements artifact candidates, update this
  file and adjust the test publisher to mirror that real schema.

## Test-Only Backend Services

The real mapping package does not implement these command backends. The test
publisher provides them only for UI workflow testing because they match
`src/simbiosys_ui/simbiosys_ui/config/rosTopics.json`.

| Service | Type | Test response |
| --- | --- | --- |
| `/mapping/start` | `std_srvs/srv/Trigger` | `success: true`, `message: "test mapping started"` |
| `/mapping/done` | `std_srvs/srv/Trigger` | `success: true`, `message: "test mapping finalized"` |
| `/mapping/save_safe_map` | `std_srvs/srv/Trigger` | `success: true`, `message: "test safe map save received"` |

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

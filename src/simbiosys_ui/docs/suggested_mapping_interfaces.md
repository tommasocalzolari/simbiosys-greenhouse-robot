# Suggested Mapping Interfaces

These are suggested non-UI backend contracts for a later implementation. The UI
does not synthesize them today.

## Start Mapping

- Service: `/mapping/start`
- Type: `std_srvs/srv/Trigger`
- Behavior: start or verify the real SLAM mapping stack and report readiness.

## Done Mapping

- Service: `/mapping/done`
- Type: `std_srvs/srv/Trigger`
- Behavior: finalize the current SLAM result and make the latest raw map and
  artifact/object candidates available.

## Artifact Candidates

- Topic: `/mapping/artifact_candidates`
- Type: a project message containing candidate ID, geometry, source, confidence,
  and optional classification.
- Required geometry: enough map-frame data for `wall`, `bed`, `obstacle`, and
  `false_scan` review. `bed` should be representable as a rectangle.

## Classify Artifact

- Service/action/topic: `/mapping/classify_artifact`
- Request: candidate ID plus one of `wall`, `bed`, `obstacle`, `false_scan`.
- Behavior: persist reviewed classification and normalize bed geometry to a
  rectangle using backend-provided geometry.

## Save Safe Map

- Service/action: `/mapping/save_safe_map`
- Request: map ID plus reviewed classifications.
- Behavior: exclude `false_scan`, persist cleaned/safe map artifacts, and return
  saved paths.

## Safe Map Output

- Topic: `/mapping/safe_map`
- Type: `nav_msgs/msg/OccupancyGrid`
- Behavior: publish the reviewed cleaned map for localization/navigation.

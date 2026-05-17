# UI Topic Contract

Config file:

```text
simbiosys_ui/config/rosTopics.json
```

| Purpose | Default topic | Type | Status |
| --- | --- | --- | --- |
| Teleop velocity | `/mirte_base_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Discovered, manual publish moved robot |
| Camera compressed | `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Discovered, preferred |
| Camera raw | `/camera/image_raw` | `sensor_msgs/msg/Image` | Discovered, fallback |
| Occupancy map | `/map` | `nav_msgs/msg/OccupancyGrid` | Configured, not available in current simulation |
| Odometry pose | `/mirte_base_controller/odom` | `nav_msgs/msg/Odometry` | Discovered |
| Localized pose | `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Assumed |
| Plant health update | `/plant_health` | `std_msgs/msg/String` containing JSON | Assumed |
| Plant health report | `/plant_health_report` | `std_msgs/msg/String` containing JSON or text | Assumed |
| Flower detections | `/flower_detections` | TBD pose/detection message | Assumed, TODO |
| Inspect bed command | `/ui/inspect_bed` | `std_msgs/msg/String` | UI-defined |
| Battery | `/battery_state` | `sensor_msgs/msg/BatteryState` | Assumed |

Operation mode is intentionally excluded from the UI.

## Plant Health JSON

Preferred flower-level payload on `/plant_health`:

```json
{
  "flower_id": "A1",
  "bed_id": "A",
  "height_cm": 31.4,
  "color": "purple",
  "health": "healthy",
  "growth_stage": "growing",
  "bug_detected": false,
  "flower_detected": true,
  "ready_for_harvest": false,
  "confidence": 0.91,
  "last_scan_time": "2026-05-15T12:00:00",
  "notes": "Normal growth"
}
```

The dashboard dummy map contains exactly three beds:

- Bed A: flowers `A1` through `A20`
- Bed B: flowers `B1` through `B18`
- Bed C: flowers `C1` through `C22`

Height and color are flower-level values. Bed panels are aggregate summaries only, including average height and counts of healthy, warning/critical, ready, and total flowers.

If an incoming `/plant_health` message has `bed_id` but no `flower_id`, it is treated as legacy bed-level data. The UI applies summary-compatible fields such as health, growth stage, confidence, bug detection, flower detection, harvest readiness, scan time, and notes to flowers in that bed, and does not crash.

## Teleop Safety

The UI publishes combined `Twist` commands while teleop buttons are held or while movement keys are pressed on the Teleop / Camera page.

Keyboard mapping:

- `W`: forward
- `S`: backward
- `A`: strafe left
- `D`: strafe right
- `Q`: rotate counter-clockwise
- `E`: rotate clockwise
- `Space`: stop / zero velocity
- `Escape`: stop / zero velocity

The UI maintains the set of active keys/buttons and computes one Twist:

- `W` and `S` contribute positive/negative `linear.x`
- `A` and `D` contribute positive/negative `linear.y`
- `Q` and `E` contribute positive/negative `angular.z`
- opposite keys cancel each other on their axis
- diagonal planar movement is normalized so its magnitude does not exceed the selected linear speed

Keyboard teleop is ignored while typing in an input, textarea, select, or editable element.

The UI publishes zero velocity when:

- A direction button is released.
- A movement key is released and no movement remains.
- A pointer leaves or cancels a direction button.
- The user navigates back to the dashboard.
- The page is hidden or unloaded.
- The active command times out.
- The UI node shuts down.

Speed modes:

- slow: `0.15 m/s`, `0.4 rad/s`
- normal: `0.30 m/s`, `0.7 rad/s`
- fast: `0.50 m/s`, `1.0 rad/s`

Linear x/y are clamped to `1.0 m/s`. Angular z is clamped to `1.5 rad/s`.

# UI Interface for Mapping, Localization, and Navigation

This document is for the UI developer. It lists the ROS topics, services, and
actions needed to display the map, set the initial localization pose, send
navigation goals, and draw the robot position and heading on the map.

The UI should not implement SLAM, localization, or planning. It should start the
appropriate launch file, or assume it is already running, and then interact with
the ROS interfaces below.

## Launch Modes

Mapping:

```bash
ros2 launch simbiosys_mapping getmap.launch.py simulation:=false
```

Localization only:

```bash
ros2 launch simbiosys_mapping localization.launch.py \
  simulation:=false \
  map:=maps/mirte_map.yaml
```

Navigation:

```bash
ros2 launch simbiosys_mapping navigation.launch.py simulation:=false
```

Use `simulation:=true` for Gazebo. Navigation already starts map loading and
AMCL, so do not start `localization.launch.py` separately when using
`navigation.launch.py`.

## Frames

Use the `map` frame for all map UI interaction.

Expected TF chain:

```text
map -> odom -> base_link
```

The map, robot pose, initial pose, goal pose, and global path are all expressed
in the `map` frame.

## Minimum UI Requirements

The UI should implement:

1. Subscribe to `/map` and draw the occupancy grid.
2. Subscribe to `/amcl_pose` or TF `map -> base_link` and draw the robot.
3. Publish `/initialpose` when the user sets the start pose.
4. Send goals through `/navigate_to_pose` action, or publish `/goal_pose`.
5. Subscribe to `/plan` and draw the planned path.
6. Show basic Nav2 health from lifecycle states or action feedback.

## Map Display

Subscribe:

```text
Topic: /map
Type: nav_msgs/msg/OccupancyGrid
Frame: map
```

Use transient local QoS if possible, because the map may be published before the
UI subscribes:

```python
QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
```

Occupancy values:

```text
-1   unknown
0    free
100  occupied
```

Important fields:

```text
msg.info.resolution
msg.info.width
msg.info.height
msg.info.origin.position.x
msg.info.origin.position.y
msg.data
```

Convert a map cell to `map` coordinates:

```text
map_x = origin_x + (cell_x + 0.5) * resolution
map_y = origin_y + (cell_y + 0.5) * resolution
```

For UI clicks, convert the clicked map pixel back to meters in the `map` frame.

## Robot Pose Display

Simple option:

```text
Topic: /amcl_pose
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Frame: map
```

Use:

```text
msg.pose.pose.position.x
msg.pose.pose.position.y
msg.pose.pose.orientation
```

For a smoother pose display, the UI backend can also listen to TF and look up:

```text
map -> base_link
```

Yaw from quaternion:

```python
import math

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
```

Quaternion from yaw:

```python
import math

def quaternion_from_yaw(yaw):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }
```

## Set Initial Pose

This is the UI equivalent of RViz `2D Pose Estimate`.

Publish:

```text
Topic: /initialpose
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Frame: map
```

Minimum content:

```text
header.frame_id = "map"
pose.pose.position.x = clicked_map_x
pose.pose.position.y = clicked_map_y
pose.pose.orientation = quaternion_from_yaw(clicked_yaw)
```

Recommended covariance for a manual click:

```python
cov = [0.0] * 36
cov[0] = 0.25
cov[7] = 0.25
cov[35] = 0.0685
```

Publish the initial pose several times, for example 5 to 10 messages at 5 Hz.
This avoids losing a single message during AMCL/Nav2 startup.

## Send Goal Pose

Preferred method:

```text
Action: /navigate_to_pose
Type: nav2_msgs/action/NavigateToPose
```

Goal content:

```text
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = goal_x
goal.pose.pose.position.y = goal_y
goal.pose.pose.orientation = quaternion_from_yaw(goal_yaw)
```

This action gives the UI feedback, result, and cancel support.

Simple RViz-compatible method:

```text
Topic: /goal_pose
Type: geometry_msgs/msg/PoseStamped
Frame: map
```

Publishing `/goal_pose` is enough for click-to-goal, but the action interface is
better for a complete UI.

## Cancel Goal

If using `/navigate_to_pose`, cancel the active action goal from the UI action
client.

Canceling a Nav2 goal stops the current navigation task. It is not the same as a
hardware emergency stop.

## Navigation Feedback

Subscribe:

```text
Topic: /plan
Type: nav_msgs/msg/Path
Frame: map
```

Draw each path pose over the map.

If using `/navigate_to_pose`, display action feedback such as current pose,
distance remaining, estimated time remaining, and number of recoveries when
available.

Useful optional overlays:

```text
/local_costmap/costmap
/global_costmap/costmap
/local_costmap/published_footprint
/global_costmap/published_footprint
/particle_cloud
/scan
```

Types:

```text
/local_costmap/costmap              nav_msgs/msg/OccupancyGrid
/global_costmap/costmap             nav_msgs/msg/OccupancyGrid
/local_costmap/published_footprint  geometry_msgs/msg/PolygonStamped
/global_costmap/published_footprint geometry_msgs/msg/PolygonStamped
/particle_cloud                     geometry_msgs/msg/PoseArray
/scan                               sensor_msgs/msg/LaserScan
```

## Mapping UI

During mapping, subscribe:

```text
/map                         nav_msgs/msg/OccupancyGrid
/scan                        sensor_msgs/msg/LaserScan
/mirte_base_controller/odom  nav_msgs/msg/Odometry
/tf                          tf2_msgs/msg/TFMessage
/tf_static                   tf2_msgs/msg/TFMessage
```

Save-map button:

```text
Service: /getmap_node/save_map
Type: std_srvs/srv/Trigger
```

Optional mapping status if `mapping_status_node` is running:

```text
Topic: /simbiosys/mapping_status
Type: simbiosys_interfaces/msg/MappingStatus
```

Fields:

```text
bool scan_seen
bool odom_seen
bool map_seen
bool localized
string active_map
string message
```

## Navigation Health

The UI backend can query lifecycle states:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

Expected state during navigation:

```text
active
```

## Optional Debug Buttons

Clear local costmap:

```text
Service: /local_costmap/clear_entirely_local_costmap
Type: nav2_msgs/srv/ClearEntireCostmap
```

Clear global costmap:

```text
Service: /global_costmap/clear_entirely_global_costmap
Type: nav2_msgs/srv/ClearEntireCostmap
```

Both services use an empty request.

## Quick Checklist

Subscribe:

```text
/map
/amcl_pose
/plan
/scan
/tf
/tf_static
/particle_cloud
/local_costmap/costmap
/global_costmap/costmap
/local_costmap/published_footprint
```

Publish:

```text
/initialpose
/goal_pose
```

Action client:

```text
/navigate_to_pose
```

Service clients:

```text
/getmap_node/save_map
/local_costmap/clear_entirely_local_costmap
/global_costmap/clear_entirely_global_costmap
```

## Common Mistakes

- Publishing initial pose or goal in the wrong frame. Use `map`.
- Forgetting orientation for initial pose or goal pose.
- Subscribing to `/map` without transient local QoS and missing the map.
- Starting navigation without setting the initial pose first.
- Treating `/amcl_pose` as valid before AMCL receives an initial pose.
- Using only `/goal_pose` and then needing cancel/progress. Use the
  `/navigate_to_pose` action for a complete UI.

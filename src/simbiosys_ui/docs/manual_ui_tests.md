# Manual UI Tests

1. Start the robot/simulation stack:

   ```bash
   cd /home/mark/MDP
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch simbiosys_bringup simulation_mirte_master.launch.py
   ```

2. Start mapping/SLAM when testing the live map:

   ```bash
   cd /home/mark/MDP
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch simbiosys_mapping getmap.launch.py simulation:=false odom_topic:=/mirte_base_controller/odom scan_topic:=/scan map_topic:=/map
   ```

3. Start the UI:

   ```bash
   cd /home/mark/MDP
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run simbiosys_ui ui_node
   ```

4. Open `http://localhost:8080`.

5. Verify the global safety button is red and labeled `STOP`.

6. In another terminal, monitor teleop:

   ```bash
   ros2 topic echo /mirte_base_controller/cmd_vel_unstamped
   ```

7. Press `STOP`; verify a zero Twist is sent and movement/navigation controls
   are disabled.

8. Press green `START`; verify UI safety is enabled but robot movement controls
   remain disabled until Take Control is pressed.

9. On Dashboard, verify bed cards show only CO2, humidity, and bugs detected.
   Missing telemetry must show unavailable.

10. Verify last scan is displayed as an age such as `2 min ago` or
    `not available`.

11. Verify no average height, confidence, flower health, or flower bed field is
    shown.

12. If `/map` exists, click a real map position and verify the selected target
    marker appears.

13. Press `Navigate`; if `simbiosys/execute_behavior` is available, verify a
    real `NAVIGATE` action goal is sent. Otherwise verify `Navigation backend
    unavailable`.

14. Change Dashboard task mode to harvest/scanning if
    `simbiosys/set_robot_mode` is available. Otherwise verify the selector is
    disabled.

15. Open Teleop / Camera.

16. Verify the camera view is smaller and the live mapping panel is visible.

17. If real map and pose are available, verify the Teleop SLAM map shows the
    robot marker.

18. Verify the Dashboard map does not show robot location.

19. Before pressing Take Control, verify movement buttons are greyed out and
    W/S/A/D/Q/E do not publish movement Twist.

20. Press `Take Control`; verify zero Twist is sent, movement controls become
    enabled, keyboard teleop is enabled, and the button changes to
    `Release Control`.

21. Press `Release Control`; verify zero Twist is sent, movement controls are
    disabled, keyboard teleop is disabled, and the button changes back to
    `Take Control`.

22. Press `STOP` while Take Control is active; verify zero Twist is sent, Take
    Control becomes inactive, and START does not automatically re-enable
    movement.

23. Press START, then Take Control, then test W/S/A/D/Q/E. Confirm opposite keys
    cancel and combinations combine.

24. Verify speeds: slow `0.50 m/s`, normal `0.75 m/s`, fast `1.00 m/s`.

25. Verify turning strengths: slow `0.8 rad/s`, normal `1.4 rad/s`, fast
    `2.0 rad/s`.

26. Switch to Arm Operations.

27. Verify movement controls are replaced by real named arm pose buttons when
    `simbiosys/send_named_arm_pose` is available; otherwise verify unavailable.

28. Press `Return to Robot Operations` and verify W/S/A/D/Q/E teleop works
    again.

29. Use the camera selector. Verify base/front camera uses real base topics and
    arm camera is disabled/unavailable unless a real arm camera topic exists.

30. Verify mapping workflow buttons remain stable: unavailable backend controls
    are disabled and no artifact candidates are created by the UI.

31. If `/mapping/artifact_candidates` is being published, verify it is
    `std_msgs/msg/String` JSON:

   ```bash
   ros2 topic echo /mapping/artifact_candidates --once --field data
   ```

32. Verify the Mapping Workflow panel shows artifact candidate count greater
    than zero after real candidate JSON is received.

33. Verify the candidate list shows received IDs such as `walls`, `bed_A`,
    `bed_B`, `bed_C`, `obstacle_1`, or `false_scan_1` when those are present in
    the topic payload.

34. Select each candidate from the list and verify the selected candidate ID,
    class hint, geometry type, and classification details update.

35. Verify renderable candidates are drawn over the real map. If a candidate has
    unknown geometry, verify it stays visible in the list and a warning is
    shown instead of crashing the UI.

36. Classify selected candidates as `wall`, `bed`, `obstacle`, and
    `false_scan`. Verify bed previews use rectangles, and classified
    `false_scan` candidates remain in the list but are hidden from the map
    preview.

37. Press `Done Mapping` after map and candidates are received. Verify the UI
    freezes the latest real map and latest real candidates from
    `/mapping/artifact_candidates`. If no candidates were received, verify the
    UI shows `No artifact candidates received`.

38. When using the standalone test publisher, verify the mapping workflow
    services exist:

   ```bash
   ros2 service list -t | grep -E "mapping|map|safe|start|done|save"
   ros2 service call /mapping/start std_srvs/srv/Trigger {}
   ros2 service call /mapping/done std_srvs/srv/Trigger {}
   ros2 service call /mapping/save_safe_map std_srvs/srv/Trigger {}
   ```

39. Verify the Mapping Workflow panel reports why a disabled button is disabled,
    for example `Start mapping service unavailable`, `No map received yet`, `No
    artifact candidates received`, or `Save safe map backend unavailable`.

40. Verify runtime diagnostics show map topic state, last map timestamp,
    artifact candidate count, Start Mapping backend state, Save Safe Map backend
    state, selected candidate ID, and review mode state.

41. Verify no dummy/mock/generated values are displayed anywhere from
    `src/simbiosys_ui`.

## Topic And Service Checks

```bash
ros2 topic list -t | grep -E "mapping|artifact|candidate|map|bed|environment"
ros2 service list -t | grep -E "mapping|map|safe|start|done|save"
ros2 topic echo /mapping/artifact_candidates --once --field data
ros2 topic echo /map --once
ros2 service call /mapping/start std_srvs/srv/Trigger {}
ros2 service call /mapping/done std_srvs/srv/Trigger {}
ros2 service call /mapping/save_safe_map std_srvs/srv/Trigger {}
```

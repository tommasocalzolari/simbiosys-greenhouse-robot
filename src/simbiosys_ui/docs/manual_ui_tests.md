# Manual UI Tests

1. Start MIRTE Gazebo simulation.

2. Start rosbridge if required:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/mark/MDP/install/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

3. Start the UI in dummy mode:

   ```bash
   cd /home/mark/MDP
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run simbiosys_ui ui_node
   ```

4. Open `http://localhost:8080`.

5. Verify the dashboard shows a visible **Digital Twin Map** / **2D Greenhouse Map** panel.

6. Verify Bed A, Bed B, and Bed C are visible as map areas.

7. Verify flower markers exist for `A1`-`A20`, `B1`-`B18`, and `C1`-`C22`.

8. Click flower `A1` and verify the detail panel shows flower-specific height, color, health, growth stage, bug status, harvest readiness, confidence, last scan, and notes.

9. Verify the bed cards show summary data only: total flowers, average height, healthy count, warning/critical count, and ready-for-harvest count.

10. Verify the concise report is computed from flower-level data and shows 3 beds and 60 flowers.

11. Click **Teleop / Camera**.

12. Run this in another terminal and verify Twist commands appear:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/mark/MDP/install/setup.bash
   ros2 topic echo /mirte_base_controller/cmd_vel_unstamped
   ```

13. Confirm the camera feed appears or the camera placeholder says it is waiting for `/camera/image_raw/compressed`.

14. Press `W` and verify the robot moves forward.

15. Release `W` and verify the robot stops.

16. Press `A` and verify the robot strafes/moves left if supported by the simulation.

17. Press `D` and verify the robot strafes/moves right if supported by the simulation.

18. Press `W + A` and verify combined forward-left movement.

19. Press `W + S` and verify they cancel in the forward/backward direction.

20. Press `Q` and verify counter-clockwise rotation.

21. Press `E` and verify clockwise rotation.

22. Press `Q + E` and verify rotation cancels.

23. Press and hold the on-screen Forward, Back, Strafe Left, Strafe Right, Rotate Left, and Rotate Right buttons and verify they publish the same combined commands as the keyboard controls.

24. Release on-screen teleop buttons and verify zero Twist is sent when no movement remains.

25. Press `Space` or `Escape` and verify zero Twist is sent.

26. Navigate back to the dashboard and verify zero Twist is sent.

27. Confirm no STOP button is visible, but stop behavior still works on release/page leave.

28. Verify keyboard teleop does not trigger while focus is in the speed selector.

29. Verify the camera panel shows a clear placeholder when no camera frames are available.

30. Publish a flower-level plant-health update and verify flower `A1` updates:

   ```bash
   ros2 topic pub --once /plant_health std_msgs/msg/String "{data: '{\"flower_id\":\"A1\",\"bed_id\":\"A\",\"height_cm\":31.4,\"color\":\"purple\",\"health\":\"healthy\",\"growth_stage\":\"growing\",\"bug_detected\":false,\"flower_detected\":true,\"ready_for_harvest\":false,\"confidence\":0.91,\"last_scan_time\":\"2026-05-15T12:00:00\",\"notes\":\"Normal growth\"}'}"
   ```

31. Publish a legacy bed-level message and verify the UI does not crash:

   ```bash
   ros2 topic pub --once /plant_health std_msgs/msg/String "{data: '{\"bed_id\":\"A\",\"health\":\"warning\",\"growth_stage\":\"ready\",\"confidence\":0.82,\"last_scan_time\":\"2026-05-15T12:05:00\",\"notes\":\"Legacy bed-level update\"}'}"
   ```

32. Stop rosbridge or leave it unavailable and verify the UI does not crash.

33. Confirm the dummy greenhouse map remains visible because the current simulation does not publish `/map`.

34. If a simulation publishes `/map`, set `dummyMode` to `false` and verify the occupancy-grid canvas can replace the dummy greenhouse map.

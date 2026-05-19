# Local Network Access

The SimBioSys UI binds to all network interfaces by default so another device on the same trusted local network can open the UI from the robot laptop.

Do not expose this UI to the public internet. Do not use router port forwarding. Only use it on a trusted local network because the UI can control the robot.

## Find the Laptop IP

```bash
hostname -I
```

Use the first LAN-looking address, for example `192.168.x.x` or `10.x.x.x`.

## Start Simulation

```bash
cd /home/mark/MDP
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch simbiosys_bringup simulation_mirte_master.launch.py
```

## Start UI

```bash
cd /home/mark/MDP
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run simbiosys_ui ui_node
```

The UI prints startup URLs such as:

```text
Local URL: http://localhost:8080
LAN URL: http://192.168.1.50:8080
```

Open this on another device on the same Wi-Fi/network:

```text
http://<LAPTOP_LAN_IP>:8080
```

The host and port can be overridden when needed:

```bash
SIMBIOSYS_UI_HOST=127.0.0.1 ros2 run simbiosys_ui ui_node
SIMBIOSYS_UI_PORT=8081 ros2 run simbiosys_ui ui_node
```

## ROS and Rosbridge Notes

The current UI backend connects to ROS directly with `rclpy`, and the browser uses relative API and camera URLs. Remote browsers therefore talk to the same laptop host that served the page.

If browser-native rosbridge support is added or used later, rosbridge must also listen on the laptop and port `9090` must be reachable from the local network. Browser-side rosbridge URLs should use the current page hostname, for example:

```text
ws://<LAPTOP_LAN_IP>:9090
```

## Firewall Troubleshooting

Do not change firewall settings unless the UI is unreachable from another device on the same network.

Check firewall status:

```bash
sudo ufw status
```

Allow the UI port if the firewall is active:

```bash
sudo ufw allow 8080/tcp
```

If rosbridge is used from the browser, also allow:

```bash
sudo ufw allow 9090/tcp
```

## Manual Network Test

1. Start the simulation on the laptop.
2. Start the UI on the laptop.
3. Run `hostname -I` and copy the first LAN IP.
4. On the laptop, open `http://localhost:8080`.
5. On a phone/tablet on the same Wi-Fi, open `http://<LAPTOP_LAN_IP>:8080`.
6. Confirm dashboard loads.
7. Go to Teleop/Camera page.
8. Confirm camera feed or camera placeholder loads.
9. Test W/A/S/D/Q/E from a laptop browser if available.
10. Test on-screen buttons from phone/tablet.
11. Confirm robot moves in simulation.
12. Confirm releasing buttons stops the robot.

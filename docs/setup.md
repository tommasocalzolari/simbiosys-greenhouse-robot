# Setup

Use the project Pixi/colcon workflow already present in this repository.

From a normal terminal, enter the repository and install dependencies:

```bash
cd /path/to/main-simbiosys
pixi install
pixi shell
```

`pixi shell` starts an interactive shell. Wait for the prompt to change, for
example to `(ro47007_mirte_ws)`, before running the next commands. If pasted
together, some terminals run later commands outside Pixi.

Inside the Pixi shell, build and source the workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build install log
colcon build
source install/setup.bash
```

Check that generated interfaces are available:

```bash
ros2 interface show simbiosys_interfaces/srv/SendNamedArmPose
```

Do not copy `mirte-documentation` into this repository. Use it as reference for
MIRTE package names, topics, and launch commands.

After each new terminal:

```bash
source install/setup.bash
```

Check package visibility:

```bash
ros2 pkg list | grep simbiosys
ros2 pkg list | grep mirte
```

If `source install/setup.bash` fails, the workspace did not build. Run
`colcon build` again and read the first package error above the summary.

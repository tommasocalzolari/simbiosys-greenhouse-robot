# Setup

Use the project Pixi/colcon workflow already present in this repository.

```bash
pixi shell
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
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

# Team Workflow

Use small branches and merge requests.

```bash
git switch main
git pull
git switch -c feature/<short-description>
```

Before a merge request:

```bash
colcon build
source install/setup.bash
ros2 launch simbiosys_bringup ui_system.launch.py
```

Rules:

- Branch from `main`.
- Do not push directly to `main`.
- Use merge requests for review.
- Keep changes focused.
- Mention manual robot/simulation steps in the merge request.
- Do not copy MIRTE documentation repositories into this repo.

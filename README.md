# autonomous_vacuum — colcon workspace

This branch is just the ROS 2 workspace wrapper around the robot itself. The project, its
documentation and its history live in the `articubot_one` package:

**→ [`src/articubot_one`](src/articubot_one)**

## Using this workspace

```bash
git clone --recurse-submodules https://github.com/samerodeh/autonomous_vaccum.git ros2_ws
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

```bash
ros2 launch articubot_one bringup_explore.launch.py
```

`build/`, `install/` and `log/` are colcon's output and are not tracked — `colcon build`
regenerates them. On Windows, note that `install/` is a symlink farm that needs elevated
privileges to recreate; this workspace is meant to be built on Linux (ROS 2 Humble).

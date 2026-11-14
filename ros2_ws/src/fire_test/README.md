# fire_test

Minimal continuous-fire node for the ROBOMINI simulation.

It publishes `VisionToGimbal` messages with `mode=2` at a configurable rate.
The existing `launcher_node` rate limit controls the real firing rate.

## Build

```bash
cd /home/wsxjr/Documents/Codex/2026-11-14/github-comltianze-devlrobomini-sim/robomini_sim/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select fire_test
source install/setup.bash
```

## Run

```bash
ros2 run fire_test continuous_fire_node
```

Specify the gimbal target and publish rate:

```bash
ros2 run fire_test continuous_fire_node --ros-args \
  -p yaw:=0.0 \
  -p pitch:=0.2 \
  -p publish_rate_hz:=20.0
```

Press `Ctrl+C` to stop firing.

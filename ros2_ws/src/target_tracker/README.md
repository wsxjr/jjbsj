# target_tracker

The `tracker_node` implements the small/big-rune state machine:

```text
CALIBRATING -> CENTERING -> READY -> AIMING -> FIRE -> RETURN -> READY
```

During calibration the gimbal remains stationary while all detected rotating
target centres are collected. A robust circle fit estimates the true rune
centre, and the new `CENTERING` state moves the gimbal onto that centre before
entering `READY`. When a new
`pending` target appears, the node predicts the target position using the
small-rune angular speed and fires once after the gimbal is aligned.

When four targets are already `active` and only one `pending` target remains,
the node skips the target-switch wait and starts aiming at the last target
immediately. The final-target condition is confirmed over several frames to
avoid reacting to a single noisy classification.

After the first shot, `continuous_after_first_shot` is enabled by default. The
tracker no longer waits for a target-switch jump: as soon as a valid `pending`
target is visible after returning to centre, it immediately enters prediction
and aiming again. Set it to `false` to restore one-shot-per-switch behaviour.

```text
/target_detections  -> select a target
/camera/camera_info -> image centre and focal lengths
/gimbal_to_vision   -> current gimbal yaw/pitch
/vision_to_gimbal   -> mode=1 aim command
```

The node never fires. It ignores targets marked `clipped` and stops controlling
when detections, camera intrinsics, or gimbal feedback become stale.

## Build

```bash
cd /home/wsxjr/Documents/Codex/2026-11-14/github-comltianze-devlrobomini-sim/robomini_sim/ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-select target_tracker
source install/setup.bash
```

## Run

Start the complete simulation, camera detector, small-rune tracker, and automatic
switch to `small` mode with one command:

```bash
ros2 launch target_tracker auto_aim.launch.py
```

Useful launch arguments:

```text
gui:=true|false
side:=blue|red
show_camera:=true|false
enable_tracker:=true|false
start_small_rune:=true|false
home_yaw_deg:=0.0
home_pitch_deg:=10.0
shot_lead_time_sec:=0.15
aim_pitch_offset_deg:=6.0
continuous_after_first_shot:=true|false
partition:=<unique name>
```

For example, run without Gazebo GUI but keep the camera window:

```bash
ros2 launch target_tracker auto_aim.launch.py gui:=false
```

Keep the rune stationary while testing target selection:

```bash
ros2 launch target_tracker auto_aim.launch.py start_small_rune:=false
```

The small rune uses angular speed `pi/3 rad/s`. The big rune uses:

```text
speed(t) = a * sin(omega * (t - t0)) + (2.090 - a)
```

The mode and instantaneous state come from `/rune/state`; `a`, `omega`, and
`t0` come from `/rune/<side>/mode`. Both rune types share the same calibrated
centre, state machine, prediction, and firing logic. The target orbit centre,
radius, and phase are sampled during calibration. The manual home angles are
only the initial sampling pose and fallback, not the final aiming centre.
`shot_lead_time_sec` is
currently `0.15` and should be tuned after testing.

Example with a calibrated rune-centre pose:

```bash
ros2 launch target_tracker auto_aim.launch.py \
  home_yaw_deg:=0.0 \
  home_pitch_deg:=10.0 \
  shot_lead_time_sec:=0.15 \
  aim_pitch_offset_deg:=6.0
```

The tracker state is published on `/tracker_state` and shown in the top-left
corner of the camera window.

Run only the tracker against an already running simulation:

```bash
ros2 run target_tracker tracker_node --ros-args -p use_sim_time:=true
```

The defaults match the camera and joint axes of the current simulation:

```text
yaw_error_sign   = -1.0
pitch_error_sign =  1.0
```

Change each axis independently if the mechanism is remounted:

```bash
ros2 run target_tracker tracker_node --ros-args \
  -p use_sim_time:=true \
  -p yaw_error_sign:=1.0 \
  -p pitch_error_sign:=1.0
```

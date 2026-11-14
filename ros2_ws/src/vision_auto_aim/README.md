# vision_auto_aim

This package currently contains one node:

```text
camera_view_node
```

It subscribes to the simulator camera, detects black circular target plates,
and classifies their blue light state from the normalized blue area ratio:

```text
blue_ratio = blue pixels inside the target circle / target circle pixels
```

Each plate is displayed with a red rectangle and a state label:

```text
off / active / pending / unknown
```

It has no gimbal control or firing logic.

## Run

Start the simulator first, then run:

```bash
ros2 run vision_auto_aim camera_view_node
```

The default image topic is:

```text
/camera/image_raw
```

Press `Q`, press `Esc`, or close the image window to stop the viewer.

The topic can be changed with a ROS parameter:

```bash
ros2 run vision_auto_aim camera_view_node \
  --ros-args -p image_topic:=/some/other/image
```

The black-circle detector can be tuned with:

```bash
ros2 run vision_auto_aim camera_view_node --ros-args \
  -p black_value_threshold:=80 \
  -p min_radius_px:=45 \
  -p max_radius_px:=70 \
  -p min_dark_fraction:=0.65 \
  -p hough_circle_threshold:=42
```

The blue-area classifier can be tuned with:

```bash
ros2 run vision_auto_aim camera_view_node --ros-args \
  -p blue_hue_min:=85 \
  -p blue_hue_max:=140 \
  -p blue_saturation_min:=100 \
  -p blue_value_min:=80 \
  -p off_blue_ratio:=0.015 \
  -p active_blue_ratio:=0.025 \
  -p pending_blue_ratio:=0.095
```

# vision_auto_aim

This package currently contains one node:

```text
camera_view_node
```

It subscribes to the simulator camera, detects black circular target plates,
draws a red rectangle around each detection, and displays the result in an
OpenCV window. It has no gimbal control or firing logic.

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

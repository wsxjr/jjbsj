# vision_auto_aim

This package currently contains one node:

```text
camera_view_node
```

It subscribes to the simulator camera and displays the latest image in an
OpenCV window. It has no target detection, gimbal control, or firing logic.

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

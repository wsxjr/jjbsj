# vision_auto_aim

`vision_auto_aim` is a Python ROS 2 package containing a starter node for the
ROBOMINI auto-aim simulation.

## Implement the detector

Edit `vision_auto_aim/auto_aim_node.py` and replace `detect_target()`:

```python
def detect_target(self, image_bgr):
    # Run the detector here.
    return Detection(u=target_x, v=target_y, confidence=score)
```

The node already handles:

- camera image and intrinsic subscriptions;
- gimbal feedback validation;
- pixel offset to absolute yaw/pitch conversion;
- gimbal limit clamping;
- stale image/feedback/target safety timeouts;
- optional firing after alignment.

The placeholders are intentionally inert. In particular, firing is disabled by
default:

```bash
ros2 run vision_auto_aim auto_aim_node
```

After the detector and control signs have been verified, enable firing with:

```bash
ros2 run vision_auto_aim auto_aim_node --ros-args -p enable_fire:=true
```

Useful topics:

| Direction | Topic | Type |
|---|---|---|
| input | `/camera/image_raw` | `sensor_msgs/Image` |
| input | `/camera/camera_info` | `sensor_msgs/CameraInfo` |
| input | `/gimbal_to_vision` | `robomini_interfaces/GimbalToVision` |
| output | `/vision_to_gimbal` | `robomini_interfaces/VisionToGimbal` |

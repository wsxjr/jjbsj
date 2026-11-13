#!/usr/bin/env python3
"""ROS 2 auto-aim node template for the ROBOMINI simulation.

The input and output topics intentionally match the vehicle interface:

* Input image:     /camera/image_raw
* Input intrinsics: /camera/camera_info
* Gimbal feedback: /gimbal_to_vision
* Command output:  /vision_to_gimbal

Only ``AutoAimNode.detect_target`` needs to be replaced for a first working
prototype. It must return the target centre in image pixels. Everything after
that step converts the pixel position into absolute yaw/pitch joint targets.

The node never fires by default. Set ``enable_fire:=true`` only after the
detector and control signs have been verified.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from robomini_interfaces.msg import GimbalToVision, VisionToGimbal

MODE_IDLE = 0
MODE_AIM = 1
MODE_AIM_AND_FIRE = 2


@dataclass(frozen=True)
class Detection:
    """One target detection in image pixels."""

    u: float
    v: float
    confidence: float = 1.0


class AutoAimNode(Node):
    """Camera-to-gimbal control skeleton.

    Replace :meth:`detect_target` with real perception code. The remaining
    methods provide the ROS plumbing and a small, safety-limited aiming loop.
    """

    def __init__(self) -> None:
        super().__init__("auto_aim_node")

        # Topics
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("gimbal_feedback_topic", "/gimbal_to_vision")
        self.declare_parameter("command_topic", "/vision_to_gimbal")

        # Loop and safety
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("image_timeout_sec", 0.25)
        self.declare_parameter("camera_info_timeout_sec", 1.0)
        self.declare_parameter("feedback_timeout_sec", 0.25)
        self.declare_parameter("lost_target_timeout_sec", 0.15)
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("enable_fire", False)
        self.declare_parameter("fire_only_when_aligned", True)
        self.declare_parameter("fire_tolerance_deg", 0.75)

        # Joint limits. These match the simulation defaults; adjust for the
        # actual mechanism if the template is reused on hardware.
        self.declare_parameter("yaw_limit_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -10.0)
        self.declare_parameter("pitch_max_deg", 45.0)

        # Sign parameters make it easy to match the simulator/camera mounting
        # convention without rewriting the geometry.
        self.declare_parameter("yaw_error_sign", 1.0)
        self.declare_parameter("pitch_error_sign", 1.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.feedback_topic = str(self.get_parameter("gimbal_feedback_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)

        control_rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.image_timeout = max(0.0, float(self.get_parameter("image_timeout_sec").value))
        self.camera_info_timeout = max(
            0.0, float(self.get_parameter("camera_info_timeout_sec").value)
        )
        self.feedback_timeout = max(
            0.0, float(self.get_parameter("feedback_timeout_sec").value)
        )
        self.lost_target_timeout = max(
            0.0, float(self.get_parameter("lost_target_timeout_sec").value)
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.enable_fire = bool(self.get_parameter("enable_fire").value)
        self.fire_only_when_aligned = bool(
            self.get_parameter("fire_only_when_aligned").value
        )
        self.fire_tolerance = math.radians(
            max(0.0, float(self.get_parameter("fire_tolerance_deg").value))
        )

        self.yaw_limit = math.radians(float(self.get_parameter("yaw_limit_deg").value))
        self.pitch_min = math.radians(float(self.get_parameter("pitch_min_deg").value))
        self.pitch_max = math.radians(float(self.get_parameter("pitch_max_deg").value))
        self.yaw_error_sign = float(self.get_parameter("yaw_error_sign").value)
        self.pitch_error_sign = float(self.get_parameter("pitch_error_sign").value)

        self.bridge = CvBridge()
        self._image: Optional[np.ndarray] = None
        self._image_time = -1.0e9
        self._camera_info_time = -1.0e9
        self._feedback_time = -1.0e9
        self._feedback: Optional[GimbalToVision] = None
        self._last_detection_time = -1.0e9
        self._last_target_yaw = 0.0
        self._last_target_pitch = 0.0
        self._commanding = False

        self.pub_command = self.create_publisher(
            VisionToGimbal, self.command_topic, 10
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GimbalToVision,
            self.feedback_topic,
            self._on_gimbal_feedback,
            10,
        )
        self.create_timer(1.0 / control_rate, self._control_loop)

        # Camera model, populated from CameraInfo.
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0

        self.get_logger().info(
            "auto_aim_node started: "
            f"image={self.image_topic}, feedback={self.feedback_topic}, "
            f"command={self.command_topic}, "
            f"control={control_rate:.0f}Hz, enable_fire={self.enable_fire}"
        )
        self.get_logger().warning(
            "detect_target() is still a placeholder; implement it in "
            "vision_auto_aim/auto_aim_node.py"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1.0e-9

    def _on_image(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"camera image conversion failed: {exc}")
            return
        self._image = np.asarray(image)
        self._image_time = self._now()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
            return
        if fx <= 0.0 or fy <= 0.0:
            return
        self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy
        self._camera_info_time = self._now()

    def _on_gimbal_feedback(self, msg: GimbalToVision) -> None:
        self._feedback = msg
        self._feedback_time = self._now()

    # ------------------------------------------------------------------
    # Algorithm entry point
    # ------------------------------------------------------------------
    def detect_target(self, image_bgr: np.ndarray) -> Optional[Detection]:
        """Return the current target centre in image pixels.

        Replace this method with the real detector. A typical implementation
        should perform colour/threshold processing or run an object detector,
        choose exactly one valid target, and return its centre:

            return Detection(u=target_x, v=target_y, confidence=score)

        Returning ``None`` means no reliable target. The template deliberately
        returns ``None`` so the node cannot issue an accidental command before
        perception has been implemented.
        """
        del image_bgr
        return None

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        now = self._now()

        if self._image is None or now - self._image_time > self.image_timeout:
            self._become_idle("camera image is unavailable or stale")
            return
        if (
            self._camera_info_time < 0.0
            or now - self._camera_info_time > self.camera_info_timeout
        ):
            self._become_idle("camera intrinsics are unavailable or stale")
            return
        if self._feedback is None or now - self._feedback_time > self.feedback_timeout:
            self._become_idle("gimbal feedback is unavailable or stale")
            return

        detection = self.detect_target(self._image)
        if detection is not None and detection.confidence >= self.min_confidence:
            self._last_detection_time = now
            self._last_target_yaw, self._last_target_pitch = self._pixel_to_target(
                detection.u, detection.v, self._feedback
            )

        if now - self._last_detection_time > self.lost_target_timeout:
            self._become_idle("no valid target")
            return

        target_yaw = self._last_target_yaw
        target_pitch = self._last_target_pitch
        feedback = self._feedback
        aligned = (
            abs(target_yaw - float(feedback.yaw)) <= self.fire_tolerance
            and abs(target_pitch - float(feedback.pitch)) <= self.fire_tolerance
        )

        fire = self.enable_fire and (
            aligned or not self.fire_only_when_aligned
        )
        mode = MODE_AIM_AND_FIRE if fire else MODE_AIM

        command = VisionToGimbal()
        command.mode = mode
        command.yaw = float(target_yaw)
        command.pitch = float(target_pitch)
        command.yaw_vel = 0.0
        command.yaw_acc = 0.0
        command.pitch_vel = 0.0
        command.pitch_acc = 0.0
        self.pub_command.publish(command)
        self._commanding = True

    def _pixel_to_target(
        self, u: float, v: float, feedback: GimbalToVision
    ) -> tuple[float, float]:
        """Convert an image point into absolute gimbal yaw/pitch targets.

        The camera is assumed to point along the barrel. The measured gimbal
        angles provide the current direction, and the pixel offset provides a
        differential rotation from that direction.
        """
        dx_angle = math.atan2(float(u) - self._cx, self._fx)
        dy_angle = math.atan2(float(v) - self._cy, self._fy)

        target_yaw = float(feedback.yaw) + self.yaw_error_sign * dx_angle
        target_pitch = float(feedback.pitch) - self.pitch_error_sign * dy_angle
        target_yaw = max(-self.yaw_limit, min(self.yaw_limit, target_yaw))
        target_pitch = max(self.pitch_min, min(self.pitch_max, target_pitch))
        return target_yaw, target_pitch

    def _become_idle(self, reason: str) -> None:
        if self._commanding:
            idle = VisionToGimbal()
            idle.mode = MODE_IDLE
            self.pub_command.publish(idle)
            self.get_logger().info(f"auto-aim inactive: {reason}")
            self._commanding = False

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AutoAimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

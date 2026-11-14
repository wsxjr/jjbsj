#!/usr/bin/env python3
"""小/大能量机关状态机跟踪节点。

策略：

* CALIBRATING：保持当前姿态，采样所有旋转靶板的圆心；
* CENTERING：根据拟合圆把云台移动到真正的旋转中心；
* READY：停在机关中心，等待 pending 目标突然切换；
* AIMING：按当前小机关或大机关运动模型，预测射击提前量后的位置；
* FIRE：云台到位后只发送一次 mode=2；
* RETURN：回到机关中心，然后重新进入 READY。

小机关使用 pi/3 rad/s 恒定角速度；大机关使用裁判下发的 a、omega、t0
计算正弦变化角速度。两种模式共用同一个云台中心、运动和射击状态机。
"""

from __future__ import annotations

import math
import json
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from robomini_interfaces.msg import (
    GimbalToVision,
    RuneState,
    TargetDetectionArray,
    VisionToGimbal,
)

MODE_IDLE = 0
MODE_AIM = 1
MODE_FIRE = 2

STATE_CALIBRATING = "CALIBRATING"
STATE_CENTERING = "CENTERING"
STATE_READY = "READY"
STATE_AIMING = "AIMING"
STATE_FIRE = "FIRE"
STATE_RETURN = "RETURN"


class TrackerNode(Node):
    """小/大机关共用的中心待机、预测瞄准和射击状态机。"""

    def __init__(self) -> None:
        super().__init__("target_tracker_node")

        self.declare_parameter("detections_topic", "/target_detections")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("feedback_topic", "/gimbal_to_vision")
        self.declare_parameter("command_topic", "/vision_to_gimbal")
        self.declare_parameter("side", "blue")
        self.declare_parameter("rune_state_topic", "/rune/state")
        self.declare_parameter("rune_mode_topic", "/rune/blue/mode")

        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("target_state", "pending")
        self.declare_parameter("detection_timeout_sec", 0.30)
        self.declare_parameter("camera_info_timeout_sec", 1.0)
        self.declare_parameter("feedback_timeout_sec", 0.25)

        # 手动标定的机关中心云台角。
        self.declare_parameter("home_yaw_deg", 0.0)
        self.declare_parameter("home_pitch_deg", 10.0)
        self.declare_parameter("home_tolerance_deg", 1.0)

        # 小型能量机关固定角速度：pi/3 rad/s。
        self.declare_parameter("omega", math.pi / 3.0)
        # 图像坐标 y 轴向下，因此正角度与视觉上的顺时针一致。
        self.declare_parameter("omega_sign", 1.0)

        # 当前使用 0.15 秒预测提前量。
        self.declare_parameter("shot_lead_time_sec", 0.15)
        self.declare_parameter("aim_pitch_offset_deg", 6.0)
        self.declare_parameter("max_aim_time_sec", 2.0)
        self.declare_parameter("fire_tolerance_deg", 0.75)
        self.declare_parameter("fire_hold_sec", 0.10)
        self.declare_parameter("return_delay_sec", 0.10)
        self.declare_parameter("return_tolerance_deg", 0.75)

        # 标定和换靶检测。
        self.declare_parameter("calibration_duration_sec", 3.0)
        self.declare_parameter("calibration_min_samples", 8)
        self.declare_parameter("calibration_ransac_iterations", 100)
        self.declare_parameter("calibration_inlier_tolerance_px", 8.0)
        self.declare_parameter("min_orbit_radius_px", 40.0)
        self.declare_parameter("max_orbit_radius_px", 400.0)
        self.declare_parameter("max_orbit_center_offset_px", 300.0)
        self.declare_parameter("switch_jump_px", 60.0)
        self.declare_parameter("switch_confirm_frames", 3)
        self.declare_parameter("switch_reacquire_timeout_sec", 1.0)
        self.declare_parameter("final_target_active_count", 4)
        self.declare_parameter("final_target_confirm_frames", 3)
        self.declare_parameter("continuous_after_first_shot", True)

        self.declare_parameter("yaw_limit_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -10.0)
        self.declare_parameter("pitch_max_deg", 45.0)
        self.declare_parameter("yaw_error_sign", -1.0)
        self.declare_parameter("pitch_error_sign", 1.0)
        self.declare_parameter("state_topic", "/tracker_state")

        self.detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.feedback_topic = str(
            self.get_parameter("feedback_topic").value
        )
        self.command_topic = str(
            self.get_parameter("command_topic").value
        )
        self.side = str(self.get_parameter("side").value)
        if self.side not in ("blue", "red"):
            raise RuntimeError(f"side must be blue or red, got {self.side!r}")
        self.rune_state_topic = str(
            self.get_parameter("rune_state_topic").value
        )
        default_mode_topic = f"/rune/{self.side}/mode"
        mode_topic_param = str(
            self.get_parameter("rune_mode_topic").value
        )
        self.rune_mode_topic = (
            mode_topic_param
            if mode_topic_param and mode_topic_param != "/rune/blue/mode"
            else default_mode_topic
        )
        self.state_topic = str(
            self.get_parameter("state_topic").value
        )

        control_rate = max(
            1.0, float(self.get_parameter("control_rate_hz").value)
        )
        self.target_state = str(self.get_parameter("target_state").value)
        self.detection_timeout = max(
            0.0, float(self.get_parameter("detection_timeout_sec").value)
        )
        self.camera_info_timeout = max(
            0.0, float(self.get_parameter("camera_info_timeout_sec").value)
        )
        self.feedback_timeout = max(
            0.0, float(self.get_parameter("feedback_timeout_sec").value)
        )

        self.home_yaw = math.radians(
            float(self.get_parameter("home_yaw_deg").value)
        )
        self.home_pitch = math.radians(
            float(self.get_parameter("home_pitch_deg").value)
        )
        self.home_tolerance = math.radians(
            max(0.0, float(self.get_parameter("home_tolerance_deg").value))
        )
        self.omega = float(self.get_parameter("omega").value)
        self.omega_sign = float(self.get_parameter("omega_sign").value)
        self.shot_lead_time = max(
            0.0, float(self.get_parameter("shot_lead_time_sec").value)
        )
        self.aim_pitch_offset = math.radians(
            float(self.get_parameter("aim_pitch_offset_deg").value)
        )
        self.max_aim_time = max(
            0.0, float(self.get_parameter("max_aim_time_sec").value)
        )
        self.fire_tolerance = math.radians(
            max(0.0, float(self.get_parameter("fire_tolerance_deg").value))
        )
        self.fire_hold_time = max(
            0.0, float(self.get_parameter("fire_hold_sec").value)
        )
        self.return_delay = max(
            0.0, float(self.get_parameter("return_delay_sec").value)
        )
        self.return_tolerance = math.radians(
            max(0.0, float(self.get_parameter("return_tolerance_deg").value))
        )

        self.calibration_duration = max(
            0.1, float(self.get_parameter("calibration_duration_sec").value)
        )
        self.calibration_min_samples = max(
            3, int(self.get_parameter("calibration_min_samples").value)
        )
        self.calibration_ransac_iterations = max(
            1,
            int(self.get_parameter("calibration_ransac_iterations").value),
        )
        self.calibration_inlier_tolerance = max(
            0.5,
            float(self.get_parameter("calibration_inlier_tolerance_px").value),
        )
        self.min_orbit_radius = max(
            1.0, float(self.get_parameter("min_orbit_radius_px").value)
        )
        self.max_orbit_radius = max(
            self.min_orbit_radius,
            float(self.get_parameter("max_orbit_radius_px").value),
        )
        self.max_orbit_center_offset = max(
            0.0,
            float(self.get_parameter("max_orbit_center_offset_px").value),
        )
        self.switch_jump = max(
            1.0, float(self.get_parameter("switch_jump_px").value)
        )
        self.switch_confirm_frames = max(
            1, int(self.get_parameter("switch_confirm_frames").value)
        )
        self.switch_reacquire_timeout = max(
            0.0,
            float(self.get_parameter("switch_reacquire_timeout_sec").value),
        )
        self.final_target_active_count = max(
            1, int(self.get_parameter("final_target_active_count").value)
        )
        self.final_target_confirm_frames = max(
            1, int(self.get_parameter("final_target_confirm_frames").value)
        )
        self.continuous_after_first_shot = bool(
            self.get_parameter("continuous_after_first_shot").value
        )

        self.yaw_limit = math.radians(
            float(self.get_parameter("yaw_limit_deg").value)
        )
        self.pitch_min = math.radians(
            float(self.get_parameter("pitch_min_deg").value)
        )
        self.pitch_max = math.radians(
            float(self.get_parameter("pitch_max_deg").value)
        )
        self.yaw_error_sign = float(
            self.get_parameter("yaw_error_sign").value
        )
        self.pitch_error_sign = float(
            self.get_parameter("pitch_error_sign").value
        )

        self._detections: Optional[TargetDetectionArray] = None
        self._detections_time = -1.0e9
        self._detection_serial = 0
        self._processed_detection_serial = -1
        self._feedback: Optional[GimbalToVision] = None
        self._feedback_time = -1.0e9
        self._camera_info_time = -1.0e9
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._commanding = False

        self._state = STATE_CALIBRATING
        self._state_started = -1.0e9
        self._calibration_start: Optional[float] = None
        self._calibration_samples = []
        self._calibration_anchor_yaw = 0.0
        self._calibration_anchor_pitch = 0.0
        self._orbit_center_x = 0.0
        self._orbit_center_y = 0.0
        self._orbit_radius = 0.0
        self._centering_yaw = 0.0
        self._centering_pitch = 0.0

        self._previous_pending_center: Optional[Tuple[float, float]] = None
        self._switch_candidate_center: Optional[Tuple[float, float]] = None
        self._switch_candidate_count = 0
        self._last_pending_time = -1.0e9
        self._final_target_count = 0
        self._shot_count = 0
        self._switch_time = -1.0e9
        self._switch_phase = 0.0
        self._aim_yaw = 0.0
        self._aim_pitch = 0.0
        self._aim_yaw_vel = 0.0
        self._aim_pitch_vel = 0.0
        self._last_state_publish = -1.0e9
        self._motion_mode = "small"
        self._rune_speed = 0.0
        self._big_a: Optional[float] = None
        self._big_omega: Optional[float] = None
        self._big_t0: Optional[float] = None

        self.pub_command = self.create_publisher(
            VisionToGimbal,
            self.command_topic,
            10,
        )
        self.pub_state = self.create_publisher(
            String,
            self.state_topic,
            10,
        )
        self.create_subscription(
            TargetDetectionArray,
            self.detections_topic,
            self._on_detections,
            10,
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
            self._on_feedback,
            10,
        )
        self.create_subscription(
            RuneState,
            self.rune_state_topic,
            self._on_rune_state,
            10,
        )
        self.create_subscription(
            String,
            self.rune_mode_topic,
            self._on_rune_mode,
            10,
        )
        self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info(
            f"rune tracker started: side={self.side}, "
            f"home=({math.degrees(self.home_yaw):.2f}, "
            f"{math.degrees(self.home_pitch):.2f})deg, "
            f"omega={self.omega:.6f}rad/s, "
            f"lead={self.shot_lead_time:.3f}s, "
            f"state={self._state}, fire=STATE_MACHINE"
        )
        self.get_logger().warning(
            "automatic rune-centre calibration is enabled; initial home angles "
            "are used only as the sampling pose and fallback values"
        )

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1.0e-9

    def _on_detections(self, msg: TargetDetectionArray) -> None:
        self._detections = msg
        self._detections_time = self._now()
        self._detection_serial += 1

    def _on_camera_info(self, msg: CameraInfo) -> None:
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
            return
        if fx <= 0.0 or fy <= 0.0:
            return
        self._fx = fx
        self._fy = fy
        self._cx = cx
        self._cy = cy
        self._camera_info_time = self._now()

    def _on_feedback(self, msg: GimbalToVision) -> None:
        self._feedback = msg
        self._feedback_time = self._now()

    def _on_rune_state(self, msg: RuneState) -> None:
        """从裁判状态获取当前模式和瞬时角速度。"""
        if msg.mode in ("small", "big"):
            if self._motion_mode != msg.mode:
                self.get_logger().info(f"rune mode changed: {msg.mode}")
            self._motion_mode = msg.mode
        self._rune_speed = float(msg.speed)

    def _on_rune_mode(self, msg: String) -> None:
        """解析大机关命令中的 a、omega 和 t0。"""
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"cannot parse rune mode JSON: {exc}")
            return

        mode = str(data.get("mode", ""))
        if mode in ("small", "big"):
            self._motion_mode = mode
        if mode != "big":
            return

        try:
            a = float(data["a"])
            omega = float(data["omega"])
            t0 = float(data["t0"])
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid big-rune parameters: {exc}")
            return
        if not all(math.isfinite(v) for v in (a, omega, t0)):
            return
        if abs(omega) < 1.0e-9:
            return

        if (
            self._big_a != a
            or self._big_omega != omega
            or self._big_t0 != t0
        ):
            self._big_a = a
            self._big_omega = omega
            self._big_t0 = t0
            self.get_logger().info(
                f"big-rune parameters: a={a:.6f}, "
                f"omega={omega:.6f}rad/s, t0={t0:.3f}s"
            )

    def _control_loop(self) -> None:
        now = self._now()
        self._publish_state(now)
        if not self._inputs_ready(now):
            return

        new_detection = (
            self._detection_serial != self._processed_detection_serial
        )
        if new_detection:
            self._processed_detection_serial = self._detection_serial

        target = self._select_target(self._detections.detections)
        if self._state == STATE_CALIBRATING:
            self._run_calibration(now, new_detection)
        elif self._state == STATE_CENTERING:
            self._run_centering(now, target)
        elif self._state == STATE_READY:
            self._run_ready(now, target, new_detection)
        elif self._state == STATE_AIMING:
            self._run_aiming(now)
        elif self._state == STATE_FIRE:
            self._run_fire_hold(now)
        elif self._state == STATE_RETURN:
            self._run_return(now, target)

    def _inputs_ready(self, now: float) -> bool:
        if (
            self._detections is None
            or now - self._detections_time > self.detection_timeout
        ):
            self._become_idle("detection result is unavailable or stale")
            return False
        if (
            self._camera_info_time < 0.0
            or now - self._camera_info_time > self.camera_info_timeout
        ):
            self._become_idle("camera intrinsics are unavailable or stale")
            return False
        if (
            self._feedback is None
            or now - self._feedback_time > self.feedback_timeout
        ):
            self._become_idle("gimbal feedback is unavailable or stale")
            return False
        return True

    def _run_calibration(
        self,
        now: float,
        new_detection: bool,
    ) -> None:
        if self._calibration_start is None:
            self._calibration_start = now
            self._calibration_anchor_yaw = float(self._feedback.yaw)
            self._calibration_anchor_pitch = float(self._feedback.pitch)
            self._calibration_samples.clear()
            self.get_logger().info(
                "automatic centering started: sampling rotating targets"
            )

        # 保持当前实际姿态不动，以当前姿态作为像素到角度的参考。
        self._publish_aim(
            self._calibration_anchor_yaw,
            self._calibration_anchor_pitch,
        )

        if new_detection:
            for target in self._detections.detections:
                if target.clipped:
                    continue
                distance_from_centre = math.hypot(
                    float(target.center_x) - self._cx,
                    float(target.center_y) - self._cy,
                )
                if (
                    self.min_orbit_radius
                    <= distance_from_centre
                    <= self.max_orbit_radius
                ):
                    self._calibration_samples.append(
                        (float(target.center_x), float(target.center_y))
                    )

        elapsed = now - self._calibration_start
        if (
            elapsed < self.calibration_duration
            or len(self._calibration_samples) < self.calibration_min_samples
        ):
            return
        if not self._finish_calibration():
            return

        self._centering_yaw = (
            self._calibration_anchor_yaw
            + self.yaw_error_sign
            * math.atan2(self._orbit_center_x - self._cx, self._fx)
        )
        self._centering_pitch = (
            self._calibration_anchor_pitch
            - self.pitch_error_sign
            * math.atan2(self._orbit_center_y - self._cy, self._fy)
        )
        self._centering_yaw = max(
            -self.yaw_limit,
            min(self.yaw_limit, self._centering_yaw),
        )
        self._centering_pitch = max(
            self.pitch_min,
            min(self.pitch_max, self._centering_pitch),
        )
        self._enter_state(STATE_CENTERING, now)
        self.get_logger().info(
            f"circle fit complete: orbit_center="
            f"({self._orbit_center_x:.1f}, {self._orbit_center_y:.1f}), "
            f"radius={self._orbit_radius:.1f}px, "
            f"samples={len(self._calibration_samples)}; centering gimbal to "
            f"({math.degrees(self._centering_yaw):.2f}, "
            f"{math.degrees(self._centering_pitch):.2f})deg"
        )

    def _finish_calibration(self) -> bool:
        """用所有旋转靶板中心拟合共同运动圆。"""
        points = np.asarray(self._calibration_samples, dtype=np.float64)
        if points.shape[0] < self.calibration_min_samples:
            return False

        fitted = self._fit_circle_ransac(points)
        if fitted is None:
            fitted = self._fit_circle_least_squares(points)
        if fitted is None:
            self.get_logger().warning(
                "calibration rejected: could not fit rotating target circle"
            )
            self._reset_calibration_window()
            return False

        center_x, center_y, radius = fitted
        if not self.min_orbit_radius <= radius <= self.max_orbit_radius:
            self.get_logger().warning(
                f"calibration rejected: radius={radius:.1f}px outside "
                f"[{self.min_orbit_radius:.1f}, {self.max_orbit_radius:.1f}]"
            )
            self._reset_calibration_window()
            return False

        self._orbit_center_x = center_x
        self._orbit_center_y = center_y
        self._orbit_radius = radius
        return True

    def _fit_circle_ransac(
        self,
        points: np.ndarray,
    ) -> Optional[Tuple[float, float, float]]:
        """随机选择三点拟合圆，用多数内点确定靶板共同旋转中心。"""
        rng = np.random.default_rng(0)
        best_inliers = None
        best_model = None

        for _ in range(self.calibration_ransac_iterations):
            indices = rng.choice(points.shape[0], size=3, replace=False)
            model = self._fit_circle_least_squares(points[indices])
            if model is None:
                continue
            center_x, center_y, radius = model
            if not self._valid_orbit_model(center_x, center_y, radius):
                continue
            distances = np.hypot(
                points[:, 0] - center_x,
                points[:, 1] - center_y,
            )
            inliers = np.abs(distances - radius) <= self.calibration_inlier_tolerance
            count = int(np.count_nonzero(inliers))
            if best_inliers is None or count > best_inliers[0]:
                best_inliers = (count, inliers)
                best_model = model

        if best_model is None or best_inliers is None or best_inliers[0] < 3:
            return None

        refined = self._fit_circle_least_squares(points[best_inliers[1]])
        if refined is None or not self._valid_orbit_model(*refined):
            return best_model
        return refined

    def _fit_circle_least_squares(
        self,
        points: np.ndarray,
    ) -> Optional[Tuple[float, float, float]]:
        """最小二乘拟合圆：x^2+y^2 = 2cx*x + 2cy*y + k。"""
        if points.shape[0] < 3:
            return None
        x = points[:, 0]
        y = points[:, 1]
        matrix = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
        rhs = x * x + y * y
        solution, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
        if rank != 3:
            return None
        center_x = float(solution[0])
        center_y = float(solution[1])
        radius_sq = float(solution[2] + center_x**2 + center_y**2)
        if radius_sq <= 0.0:
            return None
        radius = math.sqrt(radius_sq)
        if not self._valid_orbit_model(center_x, center_y, radius):
            return None
        return center_x, center_y, radius

    def _valid_orbit_model(
        self,
        center_x: float,
        center_y: float,
        radius: float,
    ) -> bool:
        if not all(math.isfinite(v) for v in (center_x, center_y, radius)):
            return False
        if not self.min_orbit_radius <= radius <= self.max_orbit_radius:
            return False
        center_offset = math.hypot(center_x - self._cx, center_y - self._cy)
        return center_offset <= self.max_orbit_center_offset

    def _reset_calibration_window(self) -> None:
        self._calibration_start = self._now()
        self._calibration_samples.clear()

    def _run_centering(self, now: float, target) -> None:
        """把云台移动到拟合出的旋转中心并等待实际到位。"""
        self._publish_aim(self._centering_yaw, self._centering_pitch)
        if not self._is_near_angles(
            self._centering_yaw,
            self._centering_pitch,
            self.home_tolerance,
        ):
            return

        self.home_yaw = self._centering_yaw
        self.home_pitch = self._centering_pitch
        # 到达后，旋转中心应位于相机光心；后续预测使用新的图像坐标系。
        self._orbit_center_x = self._cx
        self._orbit_center_y = self._cy
        self._enter_state(STATE_READY, now)
        self._set_pending_reference(target)
        self.get_logger().info(
            f"automatic centering complete: home=("
            f"{math.degrees(self.home_yaw):.2f}, "
            f"{math.degrees(self.home_pitch):.2f})deg"
        )

    def _run_ready(self, now: float, target, new_detection: bool) -> None:
        self._publish_aim(self.home_yaw, self.home_pitch)
        if target is None:
            if now - self._last_pending_time > self.switch_reacquire_timeout:
                self._previous_pending_center = None
                self._switch_candidate_center = None
                self._switch_candidate_count = 0
            return

        self._last_pending_time = now

        # 第一枪后不再等待 pending 闪现。只要识别到一个有效 pending 靶，
        # 就立刻重新进入预测瞄准；如果命中，下一帧自然选到下一阶段的新靶。
        if (
            new_detection
            and self.continuous_after_first_shot
            and self._shot_count > 0
        ):
            self._begin_aiming(target, now, "continuous pending target")
            return

        if new_detection:
            if self._detect_final_target():
                self._final_target_count += 1
                if self._final_target_count >= self.final_target_confirm_frames:
                    self._begin_aiming(target, now, "final target")
                return
            self._final_target_count = 0

        if new_detection and self._detect_target_switch(target):
            self._begin_aiming(target, now, "new pending target")

    def _detect_final_target(self) -> bool:
        """判断当前是否只剩一个待击打靶。

        已激活靶板显示两根切向蓝色灯条，未激活靶板不显示灯条。因此当稳定的
        ``active`` 数量达到四个且仍有一个 ``pending`` 时，即可确认最后靶。
        """
        active_count = sum(
            1 for target in self._detections.detections
            if target.state == "active"
        )
        pending_count = sum(
            1 for target in self._detections.detections
            if target.state == "pending" and not target.clipped
        )
        return (
            active_count >= self.final_target_active_count
            and pending_count >= 1
        )

    def _begin_aiming(self, target, now: float, reason: str) -> None:
        """锁定当前 pending 目标并进入预测瞄准状态。"""
        center = self._target_center(target)
        self._switch_phase = math.atan2(
            center[1] - self._orbit_center_y,
            center[0] - self._orbit_center_x,
        )
        self._switch_time = now
        self._enter_state(STATE_AIMING, now)
        self.get_logger().info(
            f"{reason} at pixel=({center[0]:.1f}, {center[1]:.1f}), "
            f"phase={math.degrees(self._switch_phase):.1f}deg"
        )

    def _detect_target_switch(self, target) -> bool:
        """检测 pending 中心是否突然跳到另一个靶板。

        第一个大跳变建立候选目标；随后几帧只需要新目标保持在候选中心附近，
        就继续累加确认帧数。这样不会因为新目标本身仍在旋转而反复清零。
        """
        center = self._target_center(target)
        if self._previous_pending_center is None:
            self._previous_pending_center = center
            self._switch_candidate_center = None
            self._switch_candidate_count = 0
            return False

        if self._switch_candidate_center is None:
            distance = math.hypot(
                center[0] - self._previous_pending_center[0],
                center[1] - self._previous_pending_center[1],
            )
            self._previous_pending_center = center
            if distance >= self.switch_jump:
                self._switch_candidate_center = center
                self._switch_candidate_count = 1
            return False

        candidate_distance = math.hypot(
            center[0] - self._switch_candidate_center[0],
            center[1] - self._switch_candidate_center[1],
        )
        if candidate_distance <= self.switch_jump:
            self._switch_candidate_count += 1
        else:
            self._switch_candidate_center = center
            self._switch_candidate_count = 1

        self._previous_pending_center = center
        if self._switch_candidate_count < self.switch_confirm_frames:
            return False

        self._switch_candidate_center = None
        self._switch_candidate_count = 0
        return True

    def _run_aiming(self, now: float) -> None:
        phase, phase_rate = self._predict_phase(now)
        pixel_x = self._orbit_center_x + self._orbit_radius * math.cos(phase)
        pixel_y = self._orbit_center_y + self._orbit_radius * math.sin(phase)
        (
            self._aim_yaw,
            self._aim_pitch,
            self._aim_yaw_vel,
            self._aim_pitch_vel,
        ) = self._pixel_to_home_angles(
            pixel_x,
            pixel_y,
            phase_rate=phase_rate,
        )
        self._publish_aim(
            self._aim_yaw,
            self._aim_pitch,
            self._aim_yaw_vel,
            self._aim_pitch_vel,
        )

        feedback = self._feedback
        aligned = (
            abs(self._aim_yaw - float(feedback.yaw)) <= self.fire_tolerance
            and abs(self._aim_pitch - float(feedback.pitch)) <= self.fire_tolerance
        )
        if aligned:
            self._publish_fire(self._aim_yaw, self._aim_pitch)
            self._enter_state(STATE_FIRE, now)
            self._shot_count += 1
            self.get_logger().info(
                f"fire: aim=({math.degrees(self._aim_yaw):.2f}, "
                f"{math.degrees(self._aim_pitch):.2f})deg, "
                f"shot_count={self._shot_count}"
            )
            return

        if now - self._switch_time > self.max_aim_time:
            self.get_logger().warning(
                "aim timeout; returning to rune centre"
            )
            self._enter_state(STATE_RETURN, now)

    def _predict_phase(self, now: float) -> Tuple[float, float]:
        """计算 ``now + shot_lead_time`` 时刻的目标相位和角速度。"""
        future_time = now + self.shot_lead_time
        elapsed = future_time - self._switch_time

        if self._motion_mode == "small":
            phase_rate = self.omega_sign * self.omega
            phase = self._switch_phase + phase_rate * elapsed
            return phase, phase_rate

        if (
            self._big_a is not None
            and self._big_omega is not None
            and self._big_t0 is not None
        ):
            a = self._big_a
            omega = self._big_omega
            t0 = self._big_t0
            phase_at_switch = omega * (self._switch_time - t0)
            phase_at_future = omega * (future_time - t0)
            integrated_speed = (
                (2.090 - a) * elapsed
                + (a / omega)
                * (
                    math.cos(phase_at_switch)
                    - math.cos(phase_at_future)
                )
            )
            phase = (
                self._switch_phase
                + self.omega_sign * integrated_speed
            )
            current_speed = (
                a * math.sin(phase_at_future) + (2.090 - a)
            )
            return phase, self.omega_sign * current_speed

        # 模式命令尚未到达时，使用裁判状态中的瞬时速度做短时预测。
        phase_rate = self.omega_sign * self._rune_speed
        return self._switch_phase + phase_rate * elapsed, phase_rate

    def _run_fire_hold(self, now: float) -> None:
        if now - self._state_started >= self.fire_hold_time:
            self._enter_state(STATE_RETURN, now)

    def _run_return(self, now: float, target) -> None:
        self._publish_aim(self.home_yaw, self.home_pitch)
        if now - self._state_started < self.return_delay:
            return
        if self._is_near_home(self.return_tolerance):
            if (
                self.continuous_after_first_shot
                and self._shot_count > 0
                and target is not None
                and not target.clipped
            ):
                self._begin_aiming(target, now, "continuous pending target")
                self.get_logger().info(
                    "returned to rune centre; reacquired pending target directly"
                )
            else:
                self._enter_state(STATE_READY, now)
                self._set_pending_reference(target)
                self.get_logger().info("returned to rune centre; ready")

    def _pixel_to_home_angles(
        self,
        pixel_x: float,
        pixel_y: float,
        phase_rate: float = 0.0,
    ) -> Tuple[float, float, float, float]:
        """把预测像素位置换算为绝对目标角和角速度前馈。

        ``phase_rate`` 是目标在像素圆上的角速度。位置和速度一起下发，
        可让云台伺服在跟踪匀速运动目标时不再产生固定位置误差。
        """
        dx_angle = math.atan2(pixel_x - self._cx, self._fx)
        dy_angle = math.atan2(pixel_y - self._cy, self._fy)
        yaw = self.home_yaw + self.yaw_error_sign * dx_angle
        pitch = (
            self.home_pitch
            + self.aim_pitch_offset
            - self.pitch_error_sign * dy_angle
        )

        radius = self._orbit_radius
        pixel_vx = -radius * math.sin(
            math.atan2(
                pixel_y - self._orbit_center_y,
                pixel_x - self._orbit_center_x,
            )
        ) * phase_rate
        pixel_vy = radius * math.cos(
            math.atan2(
                pixel_y - self._orbit_center_y,
                pixel_x - self._orbit_center_x,
            )
        ) * phase_rate
        dx = pixel_x - self._cx
        dy = pixel_y - self._cy
        yaw_vel = (
            self.yaw_error_sign
            * self._fx
            * pixel_vx
            / max(1.0e-9, dx * dx + self._fx * self._fx)
        )
        pitch_vel = (
            -self.pitch_error_sign
            * self._fy
            * pixel_vy
            / max(1.0e-9, dy * dy + self._fy * self._fy)
        )
        yaw = max(-self.yaw_limit, min(self.yaw_limit, yaw))
        pitch = max(self.pitch_min, min(self.pitch_max, pitch))
        return yaw, pitch, yaw_vel, pitch_vel

    def _select_target(self, detections):
        candidates = [
            target
            for target in detections
            if target.state == self.target_state and not target.clipped
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda target: (
                (float(target.center_x) - self._cx) ** 2
                + (float(target.center_y) - self._cy) ** 2
            ),
        )

    @staticmethod
    def _target_center(target) -> Tuple[float, float]:
        return float(target.center_x), float(target.center_y)

    def _set_pending_reference(self, target) -> None:
        self._previous_pending_center = (
            None if target is None else self._target_center(target)
        )
        self._switch_candidate_center = None
        self._switch_candidate_count = 0
        self._last_pending_time = (
            -1.0e9 if target is None else self._now()
        )

    def _is_near_home(self, tolerance: Optional[float] = None) -> bool:
        return self._is_near_angles(
            self.home_yaw,
            self.home_pitch,
            self.home_tolerance if tolerance is None else tolerance,
        )

    def _is_near_angles(
        self,
        yaw: float,
        pitch: float,
        tolerance: float,
    ) -> bool:
        if self._feedback is None:
            return False
        return (
            abs(float(self._feedback.yaw) - yaw) <= tolerance
            and abs(float(self._feedback.pitch) - pitch) <= tolerance
        )

    def _publish_aim(
        self,
        yaw: float,
        pitch: float,
        yaw_vel: float = 0.0,
        pitch_vel: float = 0.0,
    ) -> None:
        command = VisionToGimbal()
        command.mode = MODE_AIM
        command.yaw = float(yaw)
        command.pitch = float(pitch)
        command.yaw_vel = float(yaw_vel)
        command.pitch_vel = float(pitch_vel)
        self.pub_command.publish(command)
        self._commanding = True

    def _publish_fire(self, yaw: float, pitch: float) -> None:
        command = VisionToGimbal()
        command.mode = MODE_FIRE
        command.yaw = float(yaw)
        command.pitch = float(pitch)
        self.pub_command.publish(command)

    def _enter_state(self, state: str, now: float) -> None:
        self._state = state
        self._state_started = now
        self._final_target_count = 0
        self._publish_state(now, force=True)

    def _publish_state(self, now: float, force: bool = False) -> None:
        """周期发布状态，确保后启动的相机窗口也能收到。"""
        if not force and now - self._last_state_publish < 0.25:
            return
        message = String()
        message.data = self._state
        self.pub_state.publish(message)
        self._last_state_publish = now

    def _become_idle(self, reason: str) -> None:
        if self._commanding:
            idle = VisionToGimbal()
            idle.mode = MODE_IDLE
            self.pub_command.publish(idle)
            self.get_logger().warning(f"tracker inactive: {reason}")
        self._commanding = False
        self._state = STATE_CALIBRATING
        self._state_started = self._now()
        self._calibration_start = None
        self._calibration_anchor_yaw = 0.0
        self._calibration_anchor_pitch = 0.0
        self._calibration_samples.clear()
        self._set_pending_reference(None)
        self._final_target_count = 0
        self._shot_count = 0
        self._processed_detection_serial = -1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrackerNode()
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

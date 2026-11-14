#!/usr/bin/env python3
"""订阅相机图像，检测黑色圆形靶板并区分蓝色灯条状态。

这个节点只做一件事：

    /camera/image_raw -> 黑色圆检测 -> 蓝色面积比例 -> OpenCV 显示

每个黑色圆内部会计算：

    blue_ratio = 蓝色像素数 / 靶板圆内部总像素数

根据 blue_ratio 判断：

* off：没有蓝色灯条；
* active：有两根蓝色灯条，比例较低；
* pending：有四根蓝色灯条，比例较高；
* unknown：比例落在过渡区，暂时不强行分类。

节点不控制云台、不发布控制命令。按 Q 或 Esc，或者关闭图像窗口，都可以
结束节点。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class TargetCandidate:
    """一个黑色靶板及其蓝色面积分类结果。"""

    center_x: int
    center_y: int
    radius: int
    dark_fraction: float
    blue_ratio: float
    state: str
    clipped: bool


class CameraViewNode(Node):
    """接收相机图像，检测黑色圆并按蓝色面积比例判断灯条状态。"""

    def __init__(self) -> None:
        super().__init__("camera_view_node")

        # 图像话题默认使用仿真提供的相机原始图像。
        self.declare_parameter("image_topic", "/camera/image_raw")

        # 窗口名称同时会显示在窗口标题栏中。
        self.declare_parameter("window_name", "ROBOMINI Camera")

        # 相机可能是 60 Hz，但桌面显示 30 Hz 已经足够，也能减少界面开销。
        self.declare_parameter("display_rate_hz", 30.0)

        # 黑色亮度阈值。HSV 的 V 通道低于此值时，认为是候选黑色像素。
        self.declare_parameter("black_value_threshold", 80)

        # 当前仿真画面中靶板半径约为 48～57 px。范围保留一定余量，
        # 用于适应云台转动和靶板轻微透视变化。
        self.declare_parameter("min_radius_px", 45)
        self.declare_parameter("max_radius_px", 70)

        # 两个圆心之间的最小距离，防止同一个靶板被重复检测多次。
        self.declare_parameter("min_circle_distance_px", 80)

        # HoughCircles 的参数。圆阈值越低，检测越灵敏，但误检也越多。
        self.declare_parameter("hough_edge_threshold", 100)
        self.declare_parameter("hough_circle_threshold", 42)

        # 圆内部必须有足够比例的黑色像素，才认为是黑色靶板。
        # 这样可以过滤掉场地背景上的普通圆形边缘。
        self.declare_parameter("min_dark_fraction", 0.65)

        # 蓝色灯条的 HSV 范围。当前仿真中灯条的 H 约为 101，
        # S 和 V 都较高，因此这里采用较宽的蓝色范围。
        self.declare_parameter("blue_hue_min", 85)
        self.declare_parameter("blue_hue_max", 140)
        self.declare_parameter("blue_saturation_min", 100)
        self.declare_parameter("blue_value_min", 80)

        # 蓝色面积比例分类阈值。比例是蓝色像素数除以整个靶板圆面积，
        # 不受靶板远近造成的绝对像素面积变化影响。
        #
        # 根据当前仿真：
        #   off     ≈ 0
        #   active  ≈ 0.06
        #   pending ≈ 0.12
        self.declare_parameter("off_blue_ratio", 0.015)
        self.declare_parameter("active_blue_ratio", 0.025)
        self.declare_parameter("pending_blue_ratio", 0.095)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        display_rate = max(
            1.0, float(self.get_parameter("display_rate_hz").value)
        )
        self.black_value_threshold = int(
            self.get_parameter("black_value_threshold").value
        )
        self.min_radius = max(
            1, int(self.get_parameter("min_radius_px").value)
        )
        self.max_radius = max(
            self.min_radius, int(self.get_parameter("max_radius_px").value)
        )
        self.min_circle_distance = max(
            1, int(self.get_parameter("min_circle_distance_px").value)
        )
        self.hough_edge_threshold = max(
            1, int(self.get_parameter("hough_edge_threshold").value)
        )
        self.hough_circle_threshold = max(
            1, int(self.get_parameter("hough_circle_threshold").value)
        )
        self.min_dark_fraction = max(
            0.0,
            min(1.0, float(self.get_parameter("min_dark_fraction").value)),
        )
        self.blue_hue_min = max(
            0, min(179, int(self.get_parameter("blue_hue_min").value))
        )
        self.blue_hue_max = max(
            self.blue_hue_min,
            min(179, int(self.get_parameter("blue_hue_max").value)),
        )
        self.blue_saturation_min = max(
            0, min(255, int(self.get_parameter("blue_saturation_min").value))
        )
        self.blue_value_min = max(
            0, min(255, int(self.get_parameter("blue_value_min").value))
        )
        self.off_blue_ratio = max(
            0.0, float(self.get_parameter("off_blue_ratio").value)
        )
        self.active_blue_ratio = max(
            self.off_blue_ratio,
            float(self.get_parameter("active_blue_ratio").value),
        )
        self.pending_blue_ratio = max(
            self.active_blue_ratio,
            float(self.get_parameter("pending_blue_ratio").value),
        )

        # cv_bridge 负责把 ROS Image 消息转换成 OpenCV 可以直接显示的数组。
        self.bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._window_created = False
        self._last_circle_count = -1
        self._last_state_counts = (-1, -1, -1, -1)

        # 传感器 QoS 更适合连续图像。它优先接收最新画面，不要求每帧都可靠重传。
        self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        # 显示操作放在定时器中，避免相机每个回调都操作一次窗口。
        self.create_timer(1.0 / display_rate, self._show_latest_image)

        self.get_logger().info(
            f"camera_view_node started: image={self.image_topic}, "
            f"display={display_rate:.0f}Hz; press Q or Esc to exit"
        )

    def _on_image(self, msg: Image) -> None:
        """把 ROS 图像转换为 OpenCV BGR 图像，并保存为最新一帧。"""
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as exc:
            self.get_logger().error(f"image conversion failed: {exc}")
            return

        self._latest_image = np.asarray(image)

    def _show_latest_image(self) -> None:
        """检测黑色圆、判断蓝色灯态，并显示标注后的图像。"""
        if self._latest_image is None:
            return

        hsv = cv2.cvtColor(self._latest_image, cv2.COLOR_BGR2HSV)
        blue_mask = self._create_blue_mask(hsv)
        annotated = self._latest_image.copy()
        circles = self._detect_black_circles(hsv, blue_mask)

        state_counts = {
            "pending": 0,
            "active": 0,
            "off": 0,
            "unknown": 0,
        }
        for target in circles:
            state_counts[target.state] += 1
            cv2.rectangle(
                annotated,
                (target.center_x - target.radius, target.center_y - target.radius),
                (target.center_x + target.radius, target.center_y + target.radius),
                (0, 0, 255),
                3,
            )

            label = f"{target.state} {target.blue_ratio:.3f}"
            label_x = max(0, target.center_x - target.radius)
            label_y = target.center_y - target.radius - 8
            if label_y < 18:
                label_y = target.center_y - target.radius + 20
            cv2.putText(
                annotated,
                label,
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                self._state_label_color(target.state),
                2,
                cv2.LINE_AA,
            )

        # 只在检测数量或分类数量变化时写日志，避免 30 Hz 刷屏。
        counts = tuple(state_counts[name] for name in (
            "pending", "active", "off", "unknown"
        ))
        if len(circles) != self._last_circle_count or counts != self._last_state_counts:
            self.get_logger().info(
                f"detected black circles: {len(circles)}; "
                f"pending={state_counts['pending']}, "
                f"active={state_counts['active']}, "
                f"off={state_counts['off']}, "
                f"unknown={state_counts['unknown']}"
            )
            self._last_circle_count = len(circles)
            self._last_state_counts = counts

        try:
            if not self._window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self._window_created = True

            cv2.imshow(self.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF

            # Q 或 Esc 退出。
            if key in (ord("q"), ord("Q"), 27):
                self.get_logger().info("camera viewer exit requested")
                rclpy.shutdown()
                return

            # 点击窗口右上角关闭按钮时，OpenCV 可见性会变为 0。
            if cv2.getWindowProperty(
                self.window_name,
                cv2.WND_PROP_VISIBLE,
            ) < 1:
                self.get_logger().info("camera viewer window closed")
                rclpy.shutdown()
        except cv2.error as exc:
            # 常见原因是没有图形桌面、DISPLAY 无效，或者缺少 OpenCV GUI 支持。
            self.get_logger().error(f"cannot display camera image: {exc}")
            rclpy.shutdown()

    def _detect_black_circles(
        self,
        hsv: np.ndarray,
        blue_mask: np.ndarray,
    ) -> List[TargetCandidate]:
        """检测黑色圆形，并计算每个圆内部的蓝色面积比例。

        检测分为两步：

        1. 使用 HSV 的 V 通道和 HoughCircles 找出圆形边缘；
        2. 检查圆心附近的像素是否足够黑，过滤地板等非黑色圆形。

        对通过黑色检查的圆，再统计整个靶板圆内部的蓝色像素比例。使用比例
        而不是绝对面积，可以让远近不同的靶板使用同一组分类阈值。
        """
        value = hsv[:, :, 2]

        # 轻微模糊圆形边缘中的十字图案和抗锯齿，让 Hough 检测更稳定。
        value_blurred = cv2.GaussianBlur(value, (7, 7), 1.5)
        raw_circles = cv2.HoughCircles(
            value_blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=self.min_circle_distance,
            param1=self.hough_edge_threshold,
            param2=self.hough_circle_threshold,
            minRadius=self.min_radius,
            maxRadius=self.max_radius,
        )
        if raw_circles is None:
            return []

        height, width = value.shape
        accepted: List[TargetCandidate] = []
        for center_x, center_y, radius in np.round(raw_circles[0]).astype(int):
            # 只采样圆内部约 75% 的区域。靶板外圈和金属支架不参与判断，
            # 因此暗色比例更能代表靶板本身。
            sample_radius = max(1, int(radius * 0.75))
            left = max(0, center_x - sample_radius)
            right = min(width, center_x + sample_radius + 1)
            top = max(0, center_y - sample_radius)
            bottom = min(height, center_y + sample_radius + 1)
            if left >= right or top >= bottom:
                continue

            roi = value[top:bottom, left:right]
            yy, xx = np.ogrid[top:bottom, left:right]
            disk_mask = (
                (xx - center_x) ** 2 + (yy - center_y) ** 2
                <= sample_radius**2
            )
            pixels = roi[disk_mask]
            if pixels.size == 0:
                continue

            dark_fraction = float(
                np.count_nonzero(pixels < self.black_value_threshold)
                / pixels.size
            )
            if dark_fraction < self.min_dark_fraction:
                continue

            # 蓝色比例使用完整靶板圆作为分母。圆超出图像边界时比例会失真，
            # 因此把 clipped 标出来，并将状态设为 unknown。
            circle_left = max(0, center_x - radius)
            circle_right = min(width, center_x + radius + 1)
            circle_top = max(0, center_y - radius)
            circle_bottom = min(height, center_y + radius + 1)
            circle_yy, circle_xx = np.ogrid[
                circle_top:circle_bottom,
                circle_left:circle_right,
            ]
            full_disk_mask = (
                (circle_xx - center_x) ** 2 + (circle_yy - center_y) ** 2
                <= radius**2
            )
            circle_blue = blue_mask[
                circle_top:circle_bottom,
                circle_left:circle_right,
            ]
            circle_pixels = int(np.count_nonzero(full_disk_mask))
            if circle_pixels == 0:
                continue
            blue_pixels = int(
                np.count_nonzero(circle_blue[full_disk_mask] > 0)
            )
            blue_ratio = float(blue_pixels) / float(circle_pixels)

            clipped = (
                center_x - radius < 0
                or center_y - radius < 0
                or center_x + radius >= width
                or center_y + radius >= height
            )
            state = (
                "unknown"
                if clipped
                else self._classify_blue_ratio(blue_ratio)
            )

            accepted.append(
                TargetCandidate(
                    center_x=int(center_x),
                    center_y=int(center_y),
                    radius=int(radius),
                    dark_fraction=dark_fraction,
                    blue_ratio=blue_ratio,
                    state=state,
                    clipped=clipped,
                )
            )

        return accepted

    def _create_blue_mask(self, hsv: np.ndarray) -> np.ndarray:
        """提取蓝色灯条，并清除面积很小的零散噪声。"""
        lower = np.array(
            [self.blue_hue_min, self.blue_saturation_min, self.blue_value_min],
            dtype=np.uint8,
        )
        upper = np.array(
            [self.blue_hue_max, 255, 255],
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    def _classify_blue_ratio(self, blue_ratio: float) -> str:
        """根据归一化蓝色面积比例判断灯条状态。

        阈值之间特意留出过渡区：

        * 小于 off_blue_ratio：off
        * active_blue_ratio 到 pending_blue_ratio 之间：active
        * 大于 pending_blue_ratio：pending
        * 落在两个阈值中间的狭窄区间：unknown

        这样可以避免单帧噪声让状态在 active 和 pending 之间来回跳。
        """
        if blue_ratio >= self.pending_blue_ratio:
            return "pending"
        if blue_ratio >= self.active_blue_ratio:
            return "active"
        if blue_ratio <= self.off_blue_ratio:
            return "off"
        return "unknown"

    @staticmethod
    def _state_label_color(state: str):
        """返回不同灯态的文字颜色。红色矩形本身保持不变。"""
        if state == "pending":
            return (0, 255, 0)
        if state == "active":
            return (0, 255, 255)
        if state == "off":
            return (200, 200, 200)
        return (0, 165, 255)


def main(args=None) -> None:
    """启动节点，并在退出时清理 ROS 和 OpenCV 窗口。"""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraViewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

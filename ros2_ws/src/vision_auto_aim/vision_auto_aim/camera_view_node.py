#!/usr/bin/env python3
"""订阅相机图像，检测黑色圆形靶板并显示检测结果。

这个节点只做一件事：

    /camera/image_raw -> 黑色圆检测 -> 红色矩形框 -> OpenCV 窗口

它不检测目标、不控制云台、不发布控制命令。按 Q 或 Esc，或者关闭图像窗口，
都可以结束节点。
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewNode(Node):
    """接收相机图像，检测黑色圆形并使用红色矩形显示。"""

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

        # cv_bridge 负责把 ROS Image 消息转换成 OpenCV 可以直接显示的数组。
        self.bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._window_created = False
        self._last_circle_count = -1

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
        """检测黑色圆并显示带红色矩形框的最新图像。"""
        if self._latest_image is None:
            return

        annotated = self._latest_image.copy()
        circles = self._detect_black_circles(self._latest_image)
        for x, y, radius, _dark_fraction in circles:
            cv2.rectangle(
                annotated,
                (x - radius, y - radius),
                (x + radius, y + radius),
                (0, 0, 255),
                3,
            )

        # 只在数量变化时写日志，避免 30 Hz 刷屏。
        if len(circles) != self._last_circle_count:
            self.get_logger().info(f"detected black circles: {len(circles)}")
            self._last_circle_count = len(circles)

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
        image_bgr: np.ndarray,
    ) -> List[Tuple[int, int, int, float]]:
        """检测图像中的黑色圆形，返回圆心、半径和圆内暗色比例。

        检测分为两步：

        1. 使用 HSV 的 V 通道和 HoughCircles 找出圆形边缘；
        2. 检查圆心附近的像素是否足够黑，过滤地板等非黑色圆形。

        返回值的格式为：

            [(center_x, center_y, radius, dark_fraction), ...]
        """
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
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
        accepted: List[Tuple[int, int, int, float]] = []
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

            accepted.append(
                (int(center_x), int(center_y), int(radius), dark_fraction)
            )

        return accepted


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

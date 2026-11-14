#!/usr/bin/env python3
"""订阅相机图像并在 OpenCV 窗口中显示的 ROS 2 节点。

这个节点只做一件事：

    /camera/image_raw -> cv_bridge -> OpenCV 窗口

它不检测目标、不控制云台、不发布控制命令。按 Q 或 Esc，或者关闭图像窗口，
都可以结束节点。
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewNode(Node):
    """接收相机图像并显示最新一帧。"""

    def __init__(self) -> None:
        super().__init__("camera_view_node")

        # 图像话题默认使用仿真提供的相机原始图像。
        self.declare_parameter("image_topic", "/camera/image_raw")

        # 窗口名称同时会显示在窗口标题栏中。
        self.declare_parameter("window_name", "ROBOMINI Camera")

        # 相机可能是 60 Hz，但桌面显示 30 Hz 已经足够，也能减少界面开销。
        self.declare_parameter("display_rate_hz", 30.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        display_rate = max(
            1.0, float(self.get_parameter("display_rate_hz").value)
        )

        # cv_bridge 负责把 ROS Image 消息转换成 OpenCV 可以直接显示的数组。
        self.bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._window_created = False

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
        """显示最新图像，并处理 Q、Esc 和关闭窗口操作。"""
        if self._latest_image is None:
            return

        try:
            if not self._window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self._window_created = True

            cv2.imshow(self.window_name, self._latest_image)
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

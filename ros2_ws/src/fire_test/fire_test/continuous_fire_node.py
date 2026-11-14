#!/usr/bin/env python3
"""持续发布开火命令的测试节点。

该节点只负责向 ``/vision_to_gimbal`` 发布 ``VisionToGimbal`` 消息：

* ``mode=2``：控制云台并请求开火；
* yaw/pitch：由 ROS 参数指定，默认都是 0；
* 定时持续发布，实际射速由 ``launcher_node`` 限制为约 3 发/秒。

它不读取相机、不做目标识别，也不直接控制 Gazebo。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from robomini_interfaces.msg import VisionToGimbal

MODE_AIM_AND_FIRE = 2


class ContinuousFireNode(Node):
    """以固定频率发送 mode=2 的持续开火测试节点。"""

    def __init__(self) -> None:
        super().__init__("continuous_fire_node")

        self.declare_parameter("command_topic", "/vision_to_gimbal")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw_vel", 0.0)
        self.declare_parameter("yaw_acc", 0.0)
        self.declare_parameter("pitch_vel", 0.0)
        self.declare_parameter("pitch_acc", 0.0)

        self.command_topic = str(self.get_parameter("command_topic").value)
        publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )

        values = {
            "yaw": float(self.get_parameter("yaw").value),
            "pitch": float(self.get_parameter("pitch").value),
            "yaw_vel": float(self.get_parameter("yaw_vel").value),
            "yaw_acc": float(self.get_parameter("yaw_acc").value),
            "pitch_vel": float(self.get_parameter("pitch_vel").value),
            "pitch_acc": float(self.get_parameter("pitch_acc").value),
        }
        for name, value in values.items():
            if not math.isfinite(value):
                raise RuntimeError(f"参数 {name} 必须是有限数值，收到 {value!r}")

        self.command = VisionToGimbal()
        self.command.mode = MODE_AIM_AND_FIRE
        self.command.yaw = values["yaw"]
        self.command.pitch = values["pitch"]
        self.command.yaw_vel = values["yaw_vel"]
        self.command.yaw_acc = values["yaw_acc"]
        self.command.pitch_vel = values["pitch_vel"]
        self.command.pitch_acc = values["pitch_acc"]

        self.publisher = self.create_publisher(
            VisionToGimbal,
            self.command_topic,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_fire_command)

        self.get_logger().warning(
            f"continuous fire started: topic={self.command_topic}, "
            f"mode={self.command.mode}, yaw={self.command.yaw:.3f}, "
            f"pitch={self.command.pitch:.3f}, "
            f"publish_rate={publish_rate_hz:.0f}Hz; press Ctrl+C to stop"
        )

    def _publish_fire_command(self) -> None:
        """持续发布同一条 mode=2 命令。"""
        self.publisher.publish(self.command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ContinuousFireNode()
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

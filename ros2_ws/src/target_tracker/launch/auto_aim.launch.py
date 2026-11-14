"""一键启动 ROBOMINI 仿真、相机识别和云台跟踪节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """创建完整自瞄演示的 Launch 描述。"""
    sim_launch = os.path.join(
        get_package_share_directory("robomini_sim"),
        "launch",
        "sim.launch.py",
    )

    gui = LaunchConfiguration("gui")
    side = LaunchConfiguration("side")
    partition = LaunchConfiguration("partition")
    control_mode = LaunchConfiguration("control_mode")
    ammo_mode = LaunchConfiguration("ammo_mode")
    auto_advance = LaunchConfiguration("auto_advance")
    show_camera = LaunchConfiguration("show_camera")
    enable_tracker = LaunchConfiguration("enable_tracker")
    start_small_rune = LaunchConfiguration("start_small_rune")
    home_yaw_deg = LaunchConfiguration("home_yaw_deg")
    home_pitch_deg = LaunchConfiguration("home_pitch_deg")
    shot_lead_time_sec = LaunchConfiguration("shot_lead_time_sec")
    aim_pitch_offset_deg = LaunchConfiguration("aim_pitch_offset_deg")
    continuous_after_first_shot = LaunchConfiguration(
        "continuous_after_first_shot"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="是否打开 Gazebo GUI",
        ),
        DeclareLaunchArgument(
            "side",
            default_value="blue",
            description="控制蓝方或红方：blue / red",
        ),
        DeclareLaunchArgument(
            "partition",
            default_value="robomini",
            description="Gazebo Transport 分区，多实例运行时必须唯一",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="auto",
            description="自动或人工计分：auto / manual",
        ),
        DeclareLaunchArgument(
            "ammo_mode",
            default_value="unlimited",
            description="弹药模式：unlimited / counting",
        ),
        DeclareLaunchArgument(
            "auto_advance",
            default_value="true",
            description="完成五靶后是否自动推进阶段",
        ),
        DeclareLaunchArgument(
            "show_camera",
            default_value="true",
            description="是否打开相机识别结果显示窗口",
        ),
        DeclareLaunchArgument(
            "enable_tracker",
            default_value="true",
            description="是否启动云台跟踪节点",
        ),
        DeclareLaunchArgument(
            "start_small_rune",
            default_value="true",
            description="启动后是否自动将能量机关切换到 small 模式",
        ),
        DeclareLaunchArgument(
            "home_yaw_deg",
            default_value="0.0",
            description="机关中心对应的云台 yaw，单位度",
        ),
        DeclareLaunchArgument(
            "home_pitch_deg",
            default_value="10.0",
            description="机关中心对应的云台 pitch，单位度",
        ),
        DeclareLaunchArgument(
            "shot_lead_time_sec",
            default_value="0.15",
            description="射击提前时间，当前使用 0.15 秒",
        ),
        DeclareLaunchArgument(
            "aim_pitch_offset_deg",
            default_value="6.0",
            description="预测瞄准时额外向上的俯仰偏置，单位度",
        ),
        DeclareLaunchArgument(
            "continuous_after_first_shot",
            default_value="true",
            description="第一枪后是否直接连续瞄准当前 pending 靶",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "gui": gui,
                "side": side,
                "partition": partition,
                "control_mode": control_mode,
                "ammo_mode": ammo_mode,
                "auto_advance": auto_advance,
            }.items(),
        ),
        Node(
            package="vision_auto_aim",
            executable="camera_view_node",
            name="camera_view_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(show_camera),
        ),
        Node(
            package="target_tracker",
            executable="tracker_node",
            name="target_tracker_node",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "side": ParameterValue(side, value_type=str),
                "home_yaw_deg": ParameterValue(
                    home_yaw_deg,
                    value_type=float,
                ),
                "home_pitch_deg": ParameterValue(
                    home_pitch_deg,
                    value_type=float,
                ),
                "shot_lead_time_sec": ParameterValue(
                    shot_lead_time_sec,
                    value_type=float,
                ),
                "aim_pitch_offset_deg": ParameterValue(
                    aim_pitch_offset_deg,
                    value_type=float,
                ),
                "continuous_after_first_shot": ParameterValue(
                    continuous_after_first_shot,
                    value_type=bool,
                ),
            }],
            condition=IfCondition(enable_tracker),
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/rune/set_mode",
                        "robomini_interfaces/srv/SetRuneMode",
                        "{mode: small, a: 0.0, omega: 0.0}",
                    ],
                    output="screen",
                    condition=IfCondition(start_small_rune),
                ),
            ],
        ),
    ])

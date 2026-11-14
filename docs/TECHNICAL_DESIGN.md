# ROBOMINI 小/大能量机关视觉跟踪与射击技术文档

更新日期：2026-11-14  
适用平台：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic  
适用工程：`robomini_sim`

## 1. 系统概述

本系统在 Gazebo 仿真中完成以下闭环：

```text
相机图像
  ↓
黑色靶板检测 + 蓝色灯条状态识别
  ↓
自动拟合能量机关旋转中心并转动云台
  ↓
小/大机关运动预测
  ↓
等待目标、预测瞄准、单次开火、返回中心
```

系统同时支持：

- 小能量机关：恒定角速度；
- 大能量机关：正弦变化角速度；
- 蓝方和红方；
- 一键启动 Gazebo、识别、跟踪和自动射击；
- 首枪后直接连续处理新的 pending 靶；
- 只剩最后一个靶时跳过换靶等待。

主要代码：

- [相机识别节点](../ros2_ws/src/vision_auto_aim/vision_auto_aim/camera_view_node.py)
- [跟踪与射击状态机](../ros2_ws/src/target_tracker/target_tracker/tracker_node.py)
- [一键启动 Launch](../ros2_ws/src/target_tracker/launch/auto_aim.launch.py)
- [识别消息定义](../ros2_ws/src/robomini_interfaces/msg/TargetDetectionArray.msg)
- [单靶识别消息](../ros2_ws/src/robomini_interfaces/msg/TargetDetection.msg)

## 2. 总体架构

```text
┌──────────────────────── Gazebo ────────────────────────┐
│ 能量机关 / 发射架 / 相机 / 乒乓球 / C++ 系统插件       │
└───────────────┬───────────────────────────┬────────────┘
                │ Gazebo Transport          │
                ▼                           ▼
        ros_gz_bridge                 launcher_node
                │                           ▲
                ▼                           │
       /camera/image_raw          /vision_to_gimbal
       /camera/camera_info                 │
                │                           │
                ▼                           │
        camera_view_node ── /target_detections
                │                           │
                │                    target_tracker
                │                     ▲       │
                │                     │       │
                │              /rune/state    │
                │          /rune/<side>/mode  │
                │              /gimbal_to_vision
                │
          /tracker_state ──────────────────────────┘
                │
                ▼
       左上角显示跟踪状态
```

## 3. 功能包与职责

### 3.1 `robomini_sim`

原有仿真核心：

- 启动 Gazebo；
- ROS-Gazebo 桥接；
- 裁判状态机；
- 云台、发球和命中控制；
- 能量机关灯光和旋转。

### 3.2 `vision_auto_aim`

职责：

- 接收相机图像；
- 检测黑色圆形靶板；
- 计算蓝色灯条面积比例；
- 判断 `off / active / pending / unknown`；
- 发布结构化识别数组；
- 显示红框、灯条状态和跟踪状态。

可执行节点：

```text
camera_view_node
```

### 3.3 `target_tracker`

职责：

- 订阅识别结果；
- 自动确定能量机关中心；
- 小机关和大机关运动预测；
- 等待换靶；
- 生成云台角度和角速度前馈；
- 单次开火；
- 回中心并重新准备。

可执行节点：

```text
tracker_node
```

### 3.4 `fire_test`

独立连续开火测试包：

```text
continuous_fire_node
```

该节点不参与自动跟踪，正式运行时不要与 `tracker_node` 同时发布 `/vision_to_gimbal`。

## 4. ROS 2 接口

### 4.1 相机

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 1280×720 RGB 原始图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS | 焦距、光心和投影矩阵 |

### 4.2 识别

| 话题 | 类型 | 说明 |
|---|---|---|
| `/target_detections` | `robomini_interfaces/TargetDetectionArray` | 当前帧所有候选靶板 |

`TargetDetection` 字段：

| 字段 | 单位 | 说明 |
|---|---|---|
| `center_x` | px | 靶板圆心横坐标 |
| `center_y` | px | 靶板圆心纵坐标 |
| `radius` | px | 靶板半径 |
| `dark_fraction` | 比例 | 圆内黑色像素比例 |
| `blue_ratio` | 比例 | 蓝色灯条面积占圆形 ROI 的比例 |
| `blue_pixels` | px | 蓝色像素数 |
| `circle_pixels` | px | 圆形 ROI 总像素数 |
| `state` | 字符串 | `off / active / pending / unknown` |
| `clipped` | bool | 是否超出图像边界 |

数组消息 `TargetDetectionArray` 保留原始图像的：

- `header.stamp`；
- `header.frame_id`。

即使当前帧没有靶板，也会发布空数组。

### 4.3 跟踪状态

| 话题 | 类型 | 说明 |
|---|---|---|
| `/tracker_state` | `std_msgs/String` | `CALIBRATING / CENTERING / READY / AIMING / FIRE / RETURN` |

### 4.4 云台

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/gimbal_to_vision` | `robomini_interfaces/GimbalToVision` | 云台 → 跟踪节点 | 实际 yaw/pitch、角速度和弹数 |
| `/vision_to_gimbal` | `robomini_interfaces/VisionToGimbal` | 跟踪节点 → 云台 | 模式、目标角和前馈 |

`mode`：

```text
0 = 停止
1 = 云台瞄准，不开火
2 = 云台瞄准并请求开火
```

### 4.5 能量机关状态

| 话题 | 类型 | 说明 |
|---|---|---|
| `/rune/state` | `robomini_interfaces/RuneState` | 当前 `small/big` 模式、瞬时角速度、靶态 |
| `/rune/<side>/mode` | `std_msgs/String` JSON | 大机关 `a / omega / t0` |

大机关参数 JSON 示例：

```json
{
  "mode": "big",
  "a": 0.9,
  "omega": 1.95,
  "t0": 12.5
}
```

## 5. 黑色靶板识别

### 5.1 图像处理流程

```text
BGR 图像
  ↓
HSV 转换
  ↓
提取 V 亮度通道
  ↓
高斯模糊
  ↓
HoughCircles 圆形检测
  ↓
半径过滤
  ↓
圆内部黑色比例过滤
  ↓
候选靶板
```

当前默认参数：

| 参数 | 默认值 |
|---|---:|
| `black_value_threshold` | 80 |
| `min_radius_px` | 45 |
| `max_radius_px` | 70 |
| `min_circle_distance_px` | 80 |
| `hough_edge_threshold` | 100 |
| `hough_circle_threshold` | 42 |
| `min_dark_fraction` | 0.65 |

黑色阈值实际要求 V 足够低。OpenCV 中：

```text
V = max(R, G, B)
```

因此 `V < T` 等价于三个 RGB 通道都小于 `T`。

## 6. 蓝色灯条状态识别

在黑色圆内部进行 HSV 蓝色分割。

默认范围：

```text
H: 85 ～ 140
S: 100 ～ 255
V: 80 ～ 255
```

计算：

```text
blue_ratio = 蓝色像素数 / 靶板圆内部总像素数
```

当前分类：

```text
blue_ratio ≥ 0.095             → pending
0.025 ≤ blue_ratio < 0.095     → active
blue_ratio ≤ 0.015             → off
0.015 < blue_ratio < 0.025     → unknown
```

预期特征：

| 状态 | 灯条数量 | 蓝色比例 |
|---|---:|---:|
| `off` | 0 | 接近 0 |
| `active` | 2 | 约 0.06 |
| `pending` | 4 | 约 0.12 |

超出图像边界的靶板会被标记为 `clipped=true`，其面积比例不再可靠，因此状态设为 `unknown`。

## 7. 自动找中心

### 7.1 问题

仅靠手动配置 `home_yaw` 和 `home_pitch` 很难精确让相机光心对准机关旋转中心。中心偏差会直接影响圆周运动拟合和预测角度。

### 7.2 CALIBRATING 状态

启动后：

1. 云台保持当前实际角度不动；
2. 将该实际姿态作为像素到角度的参考；
3. 连续采样所有可见旋转黑色靶板中心；
4. 只接受距图像中心在指定半径范围内的候选；
5. 使用 RANSAC 拟合共同旋转圆；
6. 使用全部内点做最小二乘修正。

圆拟合方程：

```text
x² + y² = 2·cx·x + 2·cy·y + k
```

得到：

```text
旋转中心 = (cx, cy)
旋转半径 = sqrt(k + cx² + cy²)
```

默认采样时长：

```text
3.0 s
```

### 7.3 CENTERING 状态

根据采样时的云台姿态和拟合圆心，计算机关中心对应的绝对云台角：

```text
center_yaw =
    anchor_yaw
    + yaw_error_sign × atan2(center_x - camera_cx, fx)

center_pitch =
    anchor_pitch
    - pitch_error_sign × atan2(center_y - camera_cy, fy)
```

云台转动到该位置。实际反馈角度进入容差后：

- 更新 `home_yaw`；
- 更新 `home_pitch`；
- 将图像中的旋转中心更新为相机光心；
- 进入 `READY`。

手动传入的 `home_yaw_deg`、`home_pitch_deg` 只作为初始采样姿态和拟合失败时的回退值。

## 8. 小机关运动模型

小机关角速度恒定：

```text
omega_small = π/3 rad/s
```

目标相位：

```text
phase(t) = phase0 + direction × (π/3) × (t - t_switch)
```

预测 `shot_lead_time` 后的位置时直接使用：

```text
(t - t_switch) = now - t_switch + shot_lead_time
```

## 9. 大机关运动模型

大机关瞬时角速度：

```text
speed(t) = a × sin(omega × (t - t0)) + (2.090 - a)
```

合法范围：

```text
a     ∈ [0.780, 1.045]
omega ∈ [1.884, 2.000] rad/s
```

大机关速度始终非负，只改变快慢，不会反向。

从切换时刻 `t_switch` 到预测时刻 `t_future` 的角度增量：

```text
Δphase =
    (2.090 - a) × (t_future - t_switch)
    + (a / omega) × [
        cos(omega × (t_switch - t0))
        - cos(omega × (t_future - t0))
      ]
```

最终：

```text
phase_future = phase_at_switch + direction × Δphase
```

预测点的瞬时角速度：

```text
phase_rate_future =
    direction × [
        a × sin(omega × (t_future - t0))
        + (2.090 - a)
    ]
```

大机关的 `a、omega、t0` 从 `/rune/<side>/mode` 获取。模式尚未到达时，使用 `/rune/state.speed` 做短时近似。

## 10. 跟踪与射击状态机

```text
CALIBRATING
    ↓ 拟合共同旋转圆
CENTERING
    ↓ 云台实际对准旋转中心
READY
    ↓ 首次 pending 换靶 / 最后靶 / 首枪后连续模式
AIMING
    ↓ 云台实际角度进入火控容差
FIRE
    ↓ 只发布一次 mode=2
RETURN
    ↓ 回中心
READY 或直接 AIMING
```

### 10.1 READY

- 云台保持在机关中心；
- 默认等待 pending 目标位置跳变；
- 连续 3 帧确认后开始 AIMING；
- 如果检测到 4 个 `active` 靶和 1 个 `pending` 靶，立即处理最后靶；
- 第一枪后若 `continuous_after_first_shot=true`，不再等待闪现，直接瞄准当前 pending 靶。

### 10.2 AIMING

每个控制周期：

1. 根据当前模式计算 `now + shot_lead_time` 的目标位置；
2. 将预测像素坐标转换为绝对 yaw/pitch；
3. 增加顶部俯仰偏置；
4. 计算预测点角速度；
5. 同时发布目标角、速度前馈和加速度前馈；
6. 等待实际云台反馈进入火控容差。

当前射击参数：

```text
shot_lead_time_sec   = 0.15
aim_pitch_offset_deg = 6.0
fire_tolerance_deg   = 0.75
```

### 10.3 FIRE

只在瞄准完成时发送一次：

```text
/vision_to_gimbal mode=2
```

`launcher_node` 还会进行约 3 发/秒的射速限制。

### 10.4 RETURN

- 发送 `mode=1` 回到机关中心；
- 达到回中容差后：
  - 第一枪之前：进入 READY；
  - 第一枪之后：如果存在有效 pending 靶，直接进入 AIMING；
  - 如果暂时没有有效 pending 靶，进入 READY 等待。

## 11. 输入失效保护

以下任一数据超时会触发安全停止：

- `/target_detections` 超时；
- `/camera/camera_info` 超时；
- `/gimbal_to_vision` 超时。

处理方式：

```text
发布一次 mode=0
状态重置为 CALIBRATING
清空运动模型和标定样本
```

重新收到有效数据后重新执行自动找中心和状态机。

## 12. 主要参数

### 12.1 标定与中心

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `home_yaw_deg` | 0.0 | 初始采样 yaw / 回退值 |
| `home_pitch_deg` | 10.0 | 初始采样 pitch / 回退值 |
| `home_tolerance_deg` | 1.0 | 自动中心到位容差 |
| `calibration_duration_sec` | 3.0 | 旋转靶采样时间 |
| `calibration_min_samples` | 8 | 最少采样点 |
| `calibration_ransac_iterations` | 100 | RANSAC 次数 |
| `calibration_inlier_tolerance_px` | 8.0 | 圆内点误差 |
| `min_orbit_radius_px` | 40.0 | 最小旋转半径 |
| `max_orbit_radius_px` | 400.0 | 最大旋转半径 |

### 12.2 小机关

| 参数 | 默认值 |
|---|---:|
| `omega` | `π/3 rad/s` |
| `omega_sign` | 1.0 |

### 12.3 射击

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `shot_lead_time_sec` | 0.15 | 预测提前量 |
| `aim_pitch_offset_deg` | 6.0 | 瞄准额外抬高 |
| `max_aim_time_sec` | 2.0 | 最长瞄准时间 |
| `fire_tolerance_deg` | 0.75 | 开火角度容差 |
| `fire_hold_sec` | 0.10 | FIRE 状态保持时间 |
| `return_tolerance_deg` | 0.75 | 回中心容差 |
| `return_delay_sec` | 0.10 | 回中后等待时间 |

### 12.4 目标切换

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `switch_jump_px` | 60.0 | 目标跳变阈值 |
| `switch_confirm_frames` | 3 | 换靶确认帧数 |
| `switch_reacquire_timeout_sec` | 1.0 | pending 漏检保持时间 |
| `final_target_active_count` | 4 | 最后靶所需 active 数量 |
| `final_target_confirm_frames` | 3 | 最后靶确认帧数 |
| `continuous_after_first_shot` | true | 首枪后是否直接连续射击 |

### 12.5 云台

| 参数 | 默认值 |
|---|---:|
| `control_rate_hz` | 50.0 |
| `yaw_limit_deg` | 90.0 |
| `pitch_min_deg` | -10.0 |
| `pitch_max_deg` | 45.0 |
| `yaw_error_sign` | -1.0 |
| `pitch_error_sign` | 1.0 |

## 13. 构建

```bash
cd /home/wsxjr/Documents/Codex/2026-11-14/github-comltianze-devlrobomini-sim/robomini_sim/ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-up-to vision_auto_aim
colcon build --symlink-install --packages-select target_tracker

source install/setup.bash
```

如果修改了 `robomini_interfaces` 的 `.msg/.srv`，必须重新构建接口包：

```bash
colcon build --symlink-install \
  --packages-select robomini_interfaces vision_auto_aim target_tracker
```

## 14. 运行

### 14.1 一键启动

```bash
cd /home/wsxjr/Documents/Codex/2026-11-14/github-comltianze-devlrobomini-sim/robomini_sim/ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch target_tracker auto_aim.launch.py
```

启动内容：

- Gazebo；
- ROS-Gazebo 桥接；
- 裁判节点；
- 云台节点；
- 相机识别；
- 跟踪与射击状态机；
- 自动切换到小机关。

### 14.2 常用 Launch 参数

```bash
ros2 launch target_tracker auto_aim.launch.py \
  gui:=true \
  side:=blue \
  show_camera:=true \
  enable_tracker:=true \
  start_small_rune:=true \
  home_yaw_deg:=0.0 \
  home_pitch_deg:=10.0 \
  shot_lead_time_sec:=0.15 \
  aim_pitch_offset_deg:=6.0 \
  continuous_after_first_shot:=true
```

只启动仿真：

```bash
ros2 launch target_tracker auto_aim.launch.py \
  show_camera:=false \
  enable_tracker:=false
```

## 15. 调试方法

查看识别结果：

```bash
ros2 topic echo /target_detections --once
```

查看跟踪状态：

```bash
ros2 topic echo /tracker_state
```

查看裁判模式和角速度：

```bash
ros2 topic echo /rune/state --once
```

查看大机关参数：

```bash
ros2 topic echo /rune/blue/mode
```

查看云台命令：

```bash
ros2 topic echo /vision_to_gimbal
```

查看实际云台反馈：

```bash
ros2 topic echo /gimbal_to_vision --once
```

## 16. 已知限制

1. 自动找中心使用所有可见旋转靶板的共同圆，没有严格区分正反面的己方五个靶板。
2. 识别结果没有稳定的目标 ID，目标关联依赖位置和状态。
3. 射击提前量为固定参数，没有根据弹速、重力、距离完整求解弹道。
4. 俯仰补偿为固定角度偏置。
5. 大小机关模式参数来自仿真裁判接口，实车部署时应改为视觉估计或下位机接口。
6. 当前没有自动化单元测试和长时间连续运行测试。
7. 最近修改后已验证构建和 Launch 解析，但仍需完整实战流程测试。

## 17. 验收清单

- [ ] 启动后进入 `CALIBRATING`
- [ ] RANSAC 拟合成功并进入 `CENTERING`
- [ ] 云台到达拟合中心后进入 `READY`
- [ ] 首次 pending 目标触发 `AIMING`
- [ ] 云台对准后出现 `FIRE`
- [ ] `/gimbal_to_vision.bullet_count` 增加
- [ ] 五靶完成后自动进入大机关
- [ ] 大机关正弦速度参数正常接收
- [ ] 大机关完成五靶后自动切回小机关
- [ ] 蓝方和红方各完成一次整场测试
- [ ] `shot_lead_time_sec` 和 `aim_pitch_offset_deg` 完成标定
- [ ] 长时间运行不出现状态漂移和重复开火异常


# Apriltag Mission Trigger System

基于 ROS2 Humble 的 Apriltag 任务触发系统。

机器人前端安装 Intel RealSense D455，环境中放置多个 Apriltag，每个 Tag 对应一个任务点。
机器人检测到指定 Tag 后，自动调整姿态并运动到目标距离，然后通过 ROS2 Topic 发布任务编号给运动控制开发板。

**不依赖 SLAM，不依赖 Nav2，不依赖 Fast-LIO。Apriltag 仅用于障碍物识别、相对位姿测量和任务触发。**

---

## 目录结构

```
apriltag_mission_ws/
├── README.md
├── src/
│   ├── apriltag_interfaces/          # 自定义 ROS2 消息 (CMake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/
│   │       ├── TagPose.msg           # 单个 Tag 的 3D 位姿
│   │       ├── MissionSignal.msg     # 任务完成信号
│   │       ├── TagDetection.msg      # 单次检测结果
│   │       └── TagDetectionArray.msg # 检测结果列表
│   │
│   ├── apriltag_detector/            # Tag 检测节点 (Python)
│   │   ├── package.xml
│   │   ├── setup.py / setup.cfg
│   │   └── apriltag_detector/
│   │       ├── __init__.py
│   │       └── detector_node.py      # AprilTagDetectorNode
│   │
│   ├── visual_servo_controller/      # 视觉伺服节点 (Python)
│   │   ├── package.xml
│   │   ├── setup.py / setup.cfg
│   │   └── visual_servo_controller/
│   │       ├── __init__.py
│   │       ├── servo_node.py         # VisualServoNode
│   │       └── control_laws.py       # SerialControlLaw + ParallelControlLaw
│   │
│   ├── mission_manager/              # 任务状态机 (Python, LifecycleNode)
│   │   ├── package.xml
│   │   ├── setup.py / setup.cfg
│   │   └── mission_manager/
│   │       ├── __init__.py
│   │       ├── mission_node.py       # MissionManagerNode (LifecycleNode)
│   │       └── state_machine.py      # MissionStateMachine (纯逻辑)
│   │
│   └── robot_bringup/                # 启动文件与配置
│       ├── package.xml
│       ├── setup.py / setup.cfg
│       ├── launch/
│       │   └── bringup.launch.py     # 主启动文件
│       └── config/
│           ├── camera_tf.yaml        # base_link -> camera_link 静态 TF
│           ├── detector_params.yaml  # pupil_apriltags 配置
│           ├── controller_params.yaml# 控制律参数
│           ├── missions.yaml         # 任务队列
│           └── mission_params.yaml   # 状态机参数
```

---

## TF 树

```
base_link
    │
    └── camera_link  (静态 TF, 从 camera_tf.yaml 加载)
```

静态变换由 `apriltag_detector` 节点在启动时广播。

配置参数示例 (`camera_tf.yaml`)：

```yaml
camera_tf:
  translation:
    x: 0.18   # 相机在 base_link 前方 18cm
    y: 0.0    # 居中
    z: 0.25   # 高度 25cm
  rotation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

---

## 节点通信图

```
 /camera/color/image_raw               /imu/data
        │                                  │
        ▼                                  │
┌─────────────────┐                        │
│ apriltag_detector│                       │
│                 │   /tag_pose            │
│  publishes:     │──────┬─────────────────┤
│  /tag_pose      │      │                 │
│  /tag_detections│      ▼                 │
└─────────────────┘  ┌──────────────────┐  │
                     │visual_servo_     │  │
                     │controller        │  │
                     │                  │  │
                     │ publishes:       │  │
                     │ /cmd_vel ────────┤  │
                     └──────────────────┘  │
                              │            │
                              ▼            ▼
                     ┌──────────────────────────┐
                     │    mission_manager        │
                     │    (LifecycleNode)         │
                     │                           │
                     │  subscribes:              │
                     │    /tag_pose              │
                     │    /imu/data              │
                     │                           │
                     │  publishes:               │
                     │    /cmd_vel   (SEARCH/BLIND)│
                     │    /mission_signal         │
                     │                           │
                     │  远程参数控制:              │
                     │    detector.target_tag_id   │
                     │    servo.enable            │
                     │    servo.target_distance   │
                     └──────────────────────────┘
```

---

## 状态机流程图

```
                    ┌─────────────┐
                    │ SEARCH_TAG  │  原地旋转寻找目标Tag
                    └──────┬──────┘
                           │ Tag 被检测到 & ID 匹配
                           ▼
                    ┌─────────────┐
             ┌──────│ TRACK_TAG   │  视觉伺服跟踪
             │      └──────┬──────┘
             │             │ tag_z < prepare_distance
             │             ▼
             │      ┌─────────────┐
             │      │  PREPARE    │  接近目标，记录 IMU 航向
             │      └──┬──────┬───┘
             │         │      │ Tag 丢失
             │         │      ▼
             │         │  ┌───────────────┐
             │         │  │ BLIND_APPROACH│  盲走模式: 恒定vx + IMU航向保持
             │         │  └───────┬───────┘
             │         │          │ 时间到 / 超时
             │         ▼          ▼
             │      ┌────────────────┐
             │      │     STOP       │  停止, 短暂保持
             │      └───────┬────────┘
             │              │
             │              ▼
             │      ┌────────────────┐
             │      │  SEND_SIGNAL   │  发布 /mission_signal
             │      └───────┬────────┘
             │              │
             │              ▼
             │      ┌────────────────┐
             │      │   FINISHED     │  任务完成, 检查下一个任务
             │      └────────────────┘
             │
             └──── Tag 丢失超时 ──► 返回 SEARCH_TAG
```

### 状态转换条件

| 当前状态 | 条件 | 下一状态 |
|----------|------|----------|
| SEARCH_TAG | 目标 Tag 被检测到 | TRACK_TAG |
| SEARCH_TAG | 所有任务已完成 | FINISHED |
| TRACK_TAG | tag_z < prepare_distance | PREPARE |
| TRACK_TAG | Tag 丢失超过 tag_timeout | SEARCH_TAG |
| PREPARE | tag_z < stop_distance | STOP |
| PREPARE | Tag 丢失 | BLIND_APPROACH |
| BLIND_APPROACH | 盲走时间到 | STOP |
| BLIND_APPROACH | 超时 (安全保护) | STOP |
| STOP | 保持完成 | SEND_SIGNAL |
| SEND_SIGNAL | 信号持续 signal_duration | FINISHED |
| FINISHED | 还有下一个任务 | SEARCH_TAG |

---

## 两种机器人类型

### 类型1: 串联腿机器人 (12-DOF)

支持 Vx, Vy, Wz。

控制律：
```
error_z  = tag_z - target_distance
error_x  = tag_x
error_yaw = tag_yaw

vx = clip(kp_dist * error_z,  ±max_linear_vel)
vy = clip(kp_x   * error_x,  ±max_linear_vel)
wz = clip(kp_yaw * error_yaw, ±max_angular_vel)
```

### 类型2: 并联腿机器人 (8-DOF)

仅支持 Vx, Wz (无 Vy)。

内部子状态机: ALIGN_YAW → ALIGN_LATERAL → APPROACH

```
if |error_yaw| > threshold:   wz = kp_yaw * error_yaw
elif |error_x| > threshold:   wz = -kp_x * error_x   (旋转对中)
else:                         vx = kp_dist * error_z  (前进)
```

切换机器人类型：修改 `controller_params.yaml` 中的 `robot_type` 为 `"serial"` 或 `"parallel"`。

---

## Blind Approach 设计

当 Tag 在 PREPARE 阶段丢失时，进入盲走模式：

1. **记录参考数据**：`yaw_ref = 当前IMU航向`, `remain = last_tag_z - stop_distance`
2. **计算行进时间**：`travel_time = remain / blind_vx`
3. **控制律**：
   - `vx = blind_vx` (恒定前进速度)
   - `wz = clip(kp_yaw * (yaw_ref - yaw_current), ±max_angular_vel)`
4. **退出条件**：时间到达或 `blind_approach_timeout` 超时

IMU 只用于航向保持，不用于全局定位。

---

## 环境要求

- **操作系统**: Ubuntu 22.04
- **ROS2**: Humble Hawksbill
- **Python**: 3.10 (系统 Python, 不要使用 conda Python 3.13)
- **依赖**:
  ```bash
  # 系统 Python 包
  /usr/bin/python3.10 -m pip install pupil-apriltags scipy numpy pyyaml

  # ROS2 包 (通过 apt)
  sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-vision-msgs
  ```

---

## 编译

```bash
# 1. 确保使用系统 Python 3.10
conda deactivate

# 2. Source ROS2
source /opt/ros/humble/setup.bash

# 3. 编译
cd ~/apriltag_mission_ws
colcon build --symlink-install

# 4. Source 工作空间
source install/setup.bash
```

---

## 运行

```bash
# 启动所有节点
ros2 launch robot_bringup bringup.launch.py

# 指定机器人类型
ros2 launch robot_bringup bringup.launch.py robot_type:=parallel

# 调试模式 (详细日志)
ros2 launch robot_bringup bringup.launch.py \
    --ros-args -p log_level:=debug
```

### 手动管理 Lifecycle (可选)

```bash
# 查看状态
ros2 lifecycle get mission_manager

# 手动配置和激活
ros2 lifecycle set mission_manager configure
ros2 lifecycle set mission_manager activate

# 停止
ros2 lifecycle set mission_manager deactivate
```

---

## 配置指南

所有参数通过 YAML 文件配置，无硬编码。

### 任务配置 (`missions.yaml`)

```yaml
missions:
  - mission_id: 1
    tag_id: 0
    stop_distance: 0.8    # 在 Tag 前方 0.8m 处停止

  - mission_id: 2
    tag_id: 1
    stop_distance: 1.0
```

### 相机 TF (`camera_tf.yaml`)

配置 `base_link → camera_link` 的静态变换。

### 检测器 (`detector_params.yaml`)

pupil_apriltags 参数、Tag 物理尺寸、相机内参。

### 控制器 (`controller_params.yaml`)

`robot_type`、PID 增益、速度限制。

### 状态机 (`mission_params.yaml`)

`prepare_distance_offset`、`tag_timeout`、`blind_vx`、`blind_approach_timeout` 等。

---

## Topic 列表

| Topic | 类型 | 发布者 | 订阅者 |
|-------|------|--------|--------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense D455 | `apriltag_detector` |
| `/imu/data` | `sensor_msgs/Imu` | IMU 传感器 | `mission_manager` |
| `/tag_pose` | `apriltag_interfaces/TagPose` | `apriltag_detector` | `visual_servo_controller`, `mission_manager` |
| `/tag_detections` | `apriltag_interfaces/TagDetectionArray` | `apriltag_detector` | (调试) |
| `/cmd_vel` | `geometry_msgs/Twist` | `visual_servo_controller`, `mission_manager` | 机器人底盘 |
| `/mission_signal` | `apriltag_interfaces/MissionSignal` | `mission_manager` | 外部控制板 |

---

## 多机通信

任务完成后，`mission_manager` 发布 `/mission_signal` 消息：

```yaml
mission_id: 2    # 任务编号
tag_id: 1        # 对应的 Tag ID
distance: 0.82   # 最终测量距离 (m)
```

外部控制板订阅 `/mission_signal`，根据 `mission_id` 执行对应动作：
- `mission_id=1`: 翻越障碍
- `mission_id=2`: 爬坡
- `mission_id=3`: 绕杆
# apriltag_mission_ws

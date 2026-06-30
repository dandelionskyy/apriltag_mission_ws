# Apriltag Mission — 调参手册

## 文件索引

| 文件 | 用途 |
|------|------|
| `src/robot_bringup/config/controller_params.yaml` | 视觉伺服 (CorrectionLaw / SerialControlLaw) |
| `src/robot_bringup/config/mission_params.yaml` | 状态机 & 盲走 & 转向 |
| `src/robot_bringup/config/missions.yaml` | 任务序列 & 目标点 |
| `src/robot_bringup/config/detector_params.yaml` | Tag 检测 |
| `src/robot_bringup/config/camera_tf.yaml` | 相机外参 |

---

## 一、任务 & 目标点 (`missions.yaml`)

```yaml
missions:
  - mission_id: 1
    tag_id: 0
    stop_distance: 2.1    # target_z — 目标在 tag 前方多远 (m)
    target_x: 0.05        # 目标点横向偏置 (m), 默认 0
    steps:                # 步骤序列
      - type: approach    # 走向目标点
      - type: turn
        angle: -90        # 转向角度 (deg), 正=右转 负=左转
      - type: signal      # 发信号给控制板
```

### 目标点坐标系

以 tag 为中心, 面向 tag 时:

```
      tag
     ┌───┐
     │   │  → +X (tag 右侧)
     └───┘
       │
       ▼ +Z (tag 前方)
```

| 想停在 tag 的... | target_x |
|-----------------|----------|
| 正前方 | `0` (默认) |
| 右侧 10cm | `0.1` |
| 左侧 5cm | `-0.05` |

### 步骤类型

| type | 参数 | 说明 |
|------|------|------|
| `approach` | — | 走向 target_x / target_z 目标点 |
| `turn` | `angle` (deg) | IMU 转向, 正=右转 |
| `signal` | — | 发 TriggerMission 服务 |
| `forward` | `distance` (m) | 盲走前进 |
| `correct_heading` | `tag_id` | 旋转正对指定 tag |

---

## 二、伺服矫正 (`controller_params.yaml`)

### 算法: CorrectionLaw (parallel 模式)

边移动边修正, 不分阶段。每周期同时输出 Vx 和 Wz:

```
e_yaw = tag_yaw
e_x   = tag_x + target_x        ← target_x 是目标点横向偏置
e_z   = tag_z - target_z

damping = exp(-|e_yaw| / yaw_decay) × exp(-|e_x| / x_decay)

Vx = clip(kp_dist × e_z × damping,   ±max_linear_vel)
Wz = clip(kp_yaw × e_yaw + kp_x × e_x, ±max_angular_vel)
```

damping 效果:

```
e_yaw=0,  e_x=0    → damping = 1.0   → Vx 全速
e_yaw=yaw_decay    → damping ≈ 0.37  → Vx 降至 37%
e_yaw→∞            → damping → 0     → Vx→0, 原地旋转对准
```

### parallel 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `parallel.kp_yaw` | 1.0 | 航向修正强度 (e_yaw → Wz) |
| `parallel.kp_x` | 0.4 | 横向修正强度 (m → rad/s) |
| `parallel.kp_dist` | 0.3 | 前进速度增益 (e_z × damping → Vx) |
| `parallel.yaw_decay` | 0.3 | yaw 阻尼系数 (rad): 偏此值时 Vx 剩 37% |
| `parallel.x_decay` | 0.15 | 横向阻尼系数 (m): 偏此值时 Vx 剩 37% |
| `max_linear_vel` | 0.2 | 最大线速度 (m/s) |
| `max_angular_vel` | 0.5 | 最大角速度 (rad/s) |

### 调参顺序

**第 1 步 — kp_yaw / kp_x**

观察伺服日志 `e_yaw=... e_x=...`:

| 现象 | 调法 |
|------|------|
| 旋转太慢, 一直有 yaw 偏差 | ↑ kp_yaw |
| 旋转振荡, 反复过冲 | ↓ kp_yaw |
| 对准了航向但 tag 偏左/右, Wz 太弱 | ↑ kp_x |
| Wz 过大导致横向修正过冲 | ↓ kp_x |

**第 2 步 — 阻尼 yaw_decay / x_decay**

控制"偏多少才减速":

```
yaw_decay=0.1 → e_yaw=5.7° 时 Vx 剩 37%  (敏感: 稍偏就不走)
yaw_decay=0.3 → e_yaw=17°  时 Vx 剩 37%  (默认)
yaw_decay=0.5 → e_yaw=29°  时 Vx 剩 37%  (宽容: 偏很多还走)
```

| 现象 | 调法 |
|------|------|
| 偏差大时还在往前冲 | ↓ 对应 decay |
| 基本对准了但 Vx 太低不走 | ↑ 对应 decay |

**第 3 步 — kp_dist**

对准后前进速度:

```
kp_dist=0.3  e_z=1m  damping≈1  →  Vx=0.30 m/s
kp_dist=0.5  e_z=1m  damping≈1  →  Vx=0.50 m/s
```

### 算法: SerialControlLaw (serial 模式)

三通道独立 P 控制:

```
Vx = clip(kp_dist × (tag_z - target),  ±max_v)
Vy = clip(kp_x   × tag_x,              ±max_v)
Wz = clip(kp_yaw × tag_yaw,            ±max_w)
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `serial.kp_dist` | 0.5 | 前后速度增益 |
| `serial.kp_x` | 0.3 | 左右速度增益 |
| `serial.kp_yaw` | 1.0 | 旋转速度增益 |

---

## 三、状态机 (`mission_params.yaml`)

### SEARCH — 搜索 tag

| 参数 | 默认 | 说明 |
|------|------|------|
| `search_forward_speed` | 0.08 | 没看到 tag 时低速前进 (m/s) |
| `search_align_thresh` | 0.15 | tag 横向偏差 < 此值才进 TRACK (m) |
| `search_align_kp` | 0.4 | tag 可见但偏离时, 旋转对中的 P 增益 |

### PREPARE — 伺服逼近

| 参数 | 默认 | 说明 |
|------|------|------|
| `prepare_distance_offset` | 0.6 | 进入 PREPARE 的距离 = target_z + offset (m) |
| `prepare_arrival_thresh` | 0.10 | 深度误差 < 此值 → 到达目标 (m) |
| `tag_timeout` | 4.0 | 丢 tag 多久进入盲走 (s) |

> `prepare_arrival_thresh` 未在 yaml 声明, 默认 0.10m。需修改在 mission_params.yaml 加一行即可覆盖。

### TURN — IMU 转向

| 参数 | 默认 | 说明 |
|------|------|------|
| `turn_yaw_rate` | 0.5 | 转向角速度 (rad/s) |
| `turn_tolerance` | 0.05 | 转向完成阈值 (rad ≈ 2.9°) |
| `turn_timeout` | 15.0 | 转向超时 (s) |
| `turn_settle_time` | 0.3 | 转向前静置, 等 IMU 滤波填满 (s) |
| `turn_kp_yaw` | 1.0 | 转向 P 增益 |
| `turn_sign` | 1 | 1=正常, -1=左右互换 |

### BLIND_APPROACH — 盲走

| 参数 | 默认 | 说明 |
|------|------|------|
| `blind_vx` | 0.1 | 盲走线速度 (m/s) |
| `blind_approach_timeout` | 10.0 | 盲走超时 (s) |
| `kp_yaw` | 0.5 | 盲走航向保持 P 增益 |
| `max_angular_vel` | 0.5 | 盲走最大角速度 (rad/s) |

### DRIVE_FORWARD — 盲走前进

| 参数 | 默认 | 说明 |
|------|------|------|
| `forward_vx` | 0.15 | 前进速度 (m/s) |
| `forward_timeout` | 30.0 | 超时 (s) |

### CORRECT_HEADING — 航向修正

| 参数 | 默认 | 说明 |
|------|------|------|
| `correction_yaw_rate` | 0.3 | 搜索角速度 (rad/s) |
| `correction_tolerance` | 0.04 | 完成阈值 (rad ≈ 2.3°) |
| `correction_timeout` | 10.0 | 超时 (s) |
| `correction_kp_yaw` | 1.0 | P 增益 |

### 其他

| 参数 | 默认 | 说明 |
|------|------|------|
| `signal_duration` | 1.0 | 信号持续时间 (s) |
| `loop_rate` | 50.0 | 状态机频率 (Hz) |
| `imu_filter_window` | 20 | IMU yaw 滑动窗口 (帧数) |

---

## 四、启动

```bash
ros2 launch robot_bringup bringup.launch.py \
  robot_type:=parallel \
  turn_sign:=1
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `robot_type` | `parallel` | `serial` 或 `parallel` |
| `turn_sign` | `1` | `1`=正常, `-1`=镜像 (左转↔右转互换) |

---

## 五、典型调参场景

### 偏了还在往前冲

```yaml
# controller_params.yaml → parallel:
yaw_decay: 0.15    # ↓ (0.3→0.15)
x_decay: 0.08      # ↓
```

### 对准了但太慢

```yaml
yaw_decay: 0.5     # ↑
x_decay: 0.2       # ↑
kp_dist: 0.5       # ↑
```

### 转向转不过去

```yaml
# mission_params.yaml:
turn_kp_yaw: 1.5      # ↑ P 增益
turn_timeout: 20.0    # ↑ 放宽超时
turn_tolerance: 0.08  # ↑ 放宽精度
```

### 左转右转反了

```yaml
# mission_params.yaml:
turn_sign: -1
```

### PREPARE 卡住不进入 STOP

```yaml
# mission_params.yaml (加这行):
prepare_arrival_thresh: 0.15   # ↑ (默认 0.10)
```

### 丢 tag 太快进盲走

```yaml
# mission_params.yaml:
tag_timeout: 6.0   # ↑ (4.0→6.0)
```

### 旋转搜索太慢/太快

```yaml
# mission_params.yaml:
search_forward_speed: 0.05   # ↓ 慢一点 (默认 0.08)
search_align_kp: 0.6         # ↑ 对准更积极
```

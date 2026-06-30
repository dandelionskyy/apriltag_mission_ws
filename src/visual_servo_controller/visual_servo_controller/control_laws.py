"""
视觉伺服控制律。

- SerialControlLaw  : 串联腿 (Vx, Vy, Wz 三自由度)
- ParallelControlLaw: 并联腿 (Vx, Wz, 无 Vy)  —— 内部走 ALIGN→APPROACH 子状态机
"""

import numpy as np


class SerialControlLaw:
    """
    串联腿机器人的比例控制 (12-DOF, 支持 Vy)。

    控制律:
      error_z  = tag_z - 目标距离      → Vx = kp_dist * error_z   (前后)
      error_x  = tag_x                 → Vy = kp_x   * error_x    (左右)
      error_yaw = tag_yaw              → Wz = kp_yaw * error_yaw  (转向)
    """

    def __init__(self, kp_dist, kp_x, kp_yaw, max_v, max_w):
        self.kp_dist = kp_dist
        self.kp_x = kp_x
        self.kp_yaw = kp_yaw
        self.max_v = max_v
        self.max_w = max_w

    def compute(self, tag_x, tag_z, tag_yaw, target_dist):
        """
        输入 Tag 的 x / z / yaw 和目标距离，输出 (vx, vy, wz)
        所有速度均已限幅。
        """
        e_z = tag_z - target_dist
        e_x = tag_x
        e_yaw = tag_yaw

        vx = np.clip(self.kp_dist * e_z,   -self.max_v, self.max_v)
        vy = np.clip(self.kp_x   * e_x,    -self.max_v, self.max_v)
        wz = np.clip(self.kp_yaw * e_yaw,  -self.max_w, self.max_w)

        return float(vx), float(vy), float(wz)


class ParallelControlLaw:
    """
    并联腿机器人的比例控制 (8-DOF, 无 Vy，只有 Vx 和 Wz)。

    因为不能横向平移，所以靠旋转来对中 Tag:
      ALIGN_YAW     → 先修正航向
      ALIGN_LATERAL → 再通过旋转把 Tag 放到视野正中
      APPROACH      → 对中后直线前进

    如果前进过程中偏了，自动退回 ALIGN 重新对中。

    控制律 (与文档一致):
      if |error_x| > threshold:  wz = kp_x * error_x   (旋转对中)
      else:                      vx = kp_dist * error_z (前进)
    """

    ALIGN_YAW = 0
    ALIGN_LATERAL = 1
    APPROACH = 2
    IDLE = 3

    _STATE_NAMES = {0: 'ALIGN_YAW', 1: 'ALIGN_LATERAL', 2: 'APPROACH', 3: 'IDLE'}

    def __init__(self, kp_yaw, kp_x, kp_dist,
                 align_thresh, yaw_thresh, max_v, max_w):
        self.kp_yaw = kp_yaw
        self.kp_x = kp_x
        self.kp_dist = kp_dist
        self.align_thresh = align_thresh       # 横向对准阈值 (m)
        self.yaw_thresh = yaw_thresh           # 航向对准阈值 (rad)
        self.max_v = max_v
        self.max_w = max_w

        self.state = self.ALIGN_YAW            # 初始从修正航向开始
        self._prev_state = None                # 用于检测状态切换

    def reset(self):
        """重置子状态机，下次 compute 从头开始 ALIGN"""
        self.state = self.ALIGN_YAW
        self._prev_state = None

    @property
    def state_name(self):
        return self._STATE_NAMES.get(self.state, '?')

    def compute(self, tag_x, tag_z, tag_yaw, target_dist):
        """
        输入 Tag 的 x / z / yaw 和目标距离，输出 (vx, wz)
        返回附加信息: (vx, wz, state_changed, state_name, e_x, e_yaw, e_z)
        """
        e_z = tag_z - target_dist
        e_x = tag_x
        e_yaw = tag_yaw

        prev = self.state

        # 已经到目标距离了 → 停
        if abs(e_z) < 0.02:
            self.state = self.IDLE
            vx, wz = 0.0, 0.0
        elif self.state == self.ALIGN_YAW:
            vx, wz = self._do_align_yaw(e_yaw)
        elif self.state == self.ALIGN_LATERAL:
            vx, wz = self._do_align_lateral(e_x, e_yaw)
        elif self.state == self.APPROACH:
            vx, wz = self._do_approach(e_x, e_yaw, e_z)
        else:  # IDLE
            vx, wz = 0.0, 0.0

        changed = (self.state != prev)
        self._prev_state = prev
        return vx, wz, changed, self.state_name, e_x, e_yaw, e_z

    def _do_align_yaw(self, e_yaw):
        """修正航向：转正了才能进入横向对中"""
        if abs(e_yaw) < self.yaw_thresh:
            self.state = self.ALIGN_LATERAL
            return 0.0, 0.0
        wz = np.clip(self.kp_yaw * e_yaw, -self.max_w, self.max_w)
        return 0.0, float(wz)

    def _do_align_lateral(self, e_x, e_yaw):
        """横向对中：通过旋转让 Tag 出现在视野正中"""
        # 航向漂了？回去修
        if abs(e_yaw) >= self.yaw_thresh:
            self.state = self.ALIGN_YAW
            return 0.0, 0.0

        if abs(e_x) < self.align_thresh:
            self.state = self.APPROACH
            return 0.0, 0.0

        # e_x > 0 表示 Tag 在右边 → 右转 (wz 为负) 来对中
        wz = np.clip(-self.kp_x * e_x, -self.max_w, self.max_w)
        return 0.0, float(wz)

    def _do_approach(self, e_x, e_yaw, e_z):
        """直线前进，但随时检查是否偏了"""
        if abs(e_yaw) >= self.yaw_thresh:
            self.state = self.ALIGN_YAW
            return 0.0, 0.0
        if abs(e_x) >= self.align_thresh:
            self.state = self.ALIGN_LATERAL
            return 0.0, 0.0

        vx = np.clip(self.kp_dist * e_z, -self.max_v, self.max_v)
        return float(vx), 0.0


class CorrectionLaw:
    """
    并联腿边移动边修正控制律。

    单模式, 每周期同时输出 Vx 和 Wz, 不分阶段:

      e_yaw = tag_yaw
      e_x   = tag_x + target_x       ← target_x 为目标点横向偏置
      e_z   = tag_z - target_z

      damping = exp(-|e_yaw| / yaw_decay) * exp(-|e_x| / x_decay)
      Vx = clip(kp_dist * e_z * damping,  -max_v, max_v)
      Wz = clip(kp_yaw * e_yaw + kp_x * e_x, -max_w, max_w)

    damping 作用: 偏差大时 Vx 被压低, 优先旋转对准;
                  偏差小时 Vx 恢复, 边前进边微调。
    """

    _STATE_NAME = 'CORRECT'

    def __init__(self, kp_yaw, kp_x, kp_dist,
                 yaw_decay, x_decay, max_v, max_w):
        self.kp_yaw = kp_yaw
        self.kp_x = kp_x
        self.kp_dist = kp_dist
        self.yaw_decay = yaw_decay          # yaw 阻尼衰减系数 (rad)
        self.x_decay = x_decay              # 横向阻尼衰减系数 (m)
        self.max_v = max_v
        self.max_w = max_w
        self.target_x = 0.0                 # 外部可改写

    def reset(self):
        """接口兼容 ParallelControlLaw, 无状态可重置"""
        pass

    @property
    def state_name(self):
        return self._STATE_NAME

    def compute(self, tag_x, tag_z, tag_yaw, target_dist):
        """
        输入 Tag 的 x/z/yaw + 目标距离, 输出 (vx, wz, ...)

        返回: (vx, wz, changed, state_name, e_x, e_yaw, e_z)
               changed 始终为 False (无状态切换)
        """
        import math

        e_yaw = tag_yaw
        e_x = tag_x + self.target_x
        e_z = tag_z - target_dist

        # 阻尼: 偏差越大 Vx 越低, 优先对准
        damp_yaw = math.exp(-abs(e_yaw) / self.yaw_decay) if self.yaw_decay > 0 else 1.0
        damp_x = math.exp(-abs(e_x) / self.x_decay) if self.x_decay > 0 else 1.0
        damping = damp_yaw * damp_x

        vx = np.clip(self.kp_dist * e_z * damping, -self.max_v, self.max_v)
        wz = np.clip(self.kp_yaw * e_yaw + self.kp_x * e_x, -self.max_w, self.max_w)

        return float(vx), float(wz), False, self._STATE_NAME, e_x, e_yaw, e_z

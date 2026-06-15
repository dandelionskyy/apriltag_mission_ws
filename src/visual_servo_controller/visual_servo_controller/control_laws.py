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

    def reset(self):
        """重置子状态机，下次 compute 从头开始 ALIGN"""
        self.state = self.ALIGN_YAW

    def compute(self, tag_x, tag_z, tag_yaw, target_dist):
        """
        输入 Tag 的 x / z / yaw 和目标距离，输出 (vx, wz)
        """
        e_z = tag_z - target_dist
        e_x = tag_x
        e_yaw = tag_yaw

        # 已经到目标距离了 → 停
        if abs(e_z) < 0.02:
            self.state = self.IDLE
            return 0.0, 0.0

        # -- 子状态机 --
        if self.state == self.ALIGN_YAW:
            return self._do_align_yaw(e_yaw)

        elif self.state == self.ALIGN_LATERAL:
            return self._do_align_lateral(e_x, e_yaw)

        elif self.state == self.APPROACH:
            return self._do_approach(e_x, e_yaw, e_z)

        else:  # IDLE
            return 0.0, 0.0

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

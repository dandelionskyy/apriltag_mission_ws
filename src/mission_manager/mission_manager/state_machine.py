"""
任务状态机 —— 纯逻辑，不依赖 ROS。

状态流转:
  SEARCH_TAG → TRACK_TAG → PREPARE → BLIND_APPROACH → STOP → SEND_SIGNAL → FINISHED

MissionManagerNode (LifecycleNode) 包装这个类，负责所有 ROS 通信。
"""

import math


class MissionStateMachine:
    """纯状态机：根据输入数据评估状态转移，返回动作指令"""

    # 七个状态
    SEARCH_TAG      = "SEARCH_TAG"
    TRACK_TAG       = "TRACK_TAG"
    PREPARE         = "PREPARE"
    BLIND_APPROACH  = "BLIND_APPROACH"
    STOP            = "STOP"
    SEND_SIGNAL     = "SEND_SIGNAL"
    FINISHED        = "FINISHED"

    def __init__(self, missions, params):
        """
        missions: 任务列表，每项 {'mission_id', 'tag_id', 'stop_distance'}
        params:   状态机参数
            prepare_distance_offset  - stop_distance + offset = 进入 PREPARE 的距离
            tag_timeout             - 丢 Tag 超时 (s)
            search_yaw_rate         - 搜索时的旋转速度 (rad/s)
            blind_vx               - 盲走前进速度 (m/s)
            blind_approach_timeout - 盲走最大时长 (s)
            kp_yaw                 - 盲走航向修正 P 增益
            max_angular_vel        - 最大角速度 (rad/s)
            signal_duration        - 信号持续时间 (s)
            loop_rate              - 控制频率 (Hz)
        """
        self.missions = missions
        self.mission_idx = 0
        self.p = params

        # 当前状态
        self.state = self.SEARCH_TAG

        # 当前任务的目标
        self.target_tag_id = -1
        self.target_distance = 0.5
        self.mission_id = -1
        self._load_mission()

        # 跟踪数据
        self.last_tag = None         # (x, z, yaw, id) 最后一次看到的 Tag
        self.last_tag_time = None    # 最后一次看到 Tag 的时间
        self.imu_yaw = 0.0           # 当前 IMU 航向

        # 盲走参数
        self.blind_yaw_ref = 0.0
        self.blind_travel = 0.0      # 需要走的时间 (s)
        self.blind_start = None      # 盲走开始时间

        # STOP / SIGNAL 计时
        self.stop_start = None
        self.signal_start = None

        # 状态进入时间 (用于调试)
        self.state_enter_time = None

    # -------------------------------------------------------------------
    # 属性
    # -------------------------------------------------------------------

    @property
    def prepare_dist(self):
        """进入 PREPARE 的距离阈值 = stop_distance + offset"""
        return self.target_distance + self.p.get('prepare_distance_offset', 0.3)

    @property
    def done(self):
        """所有任务是否已完成"""
        return self.mission_idx >= len(self.missions)

    # -------------------------------------------------------------------
    # 任务管理
    # -------------------------------------------------------------------

    def _load_mission(self):
        """从队列加载当前任务"""
        if self.done:
            self.target_tag_id = -1
            self.target_distance = 0.5
            self.mission_id = -1
            return
        m = self.missions[self.mission_idx]
        self.target_tag_id   = m['tag_id']
        self.target_distance = m.get('stop_distance', 0.5)
        self.mission_id      = m.get('mission_id', self.mission_idx)

    def _next_mission(self):
        """加载下一个任务"""
        self.mission_idx += 1
        self._load_mission()

    # -------------------------------------------------------------------
    # 外部接口
    # -------------------------------------------------------------------

    def update_imu(self, yaw):
        """更新 IMU 航向 (由节点回调调用)"""
        self.imu_yaw = yaw

    def evaluate(self, tag, now):
        """
        每个控制周期调用一次。

        tag: (x, z, yaw, id) 或 None (当前没有检测到)
        now: 当前时间 (秒, time.monotonic())

        返回 dict:
          action         - 日志字符串
          cmd_vx, cmd_vy, cmd_wz - 速度指令
          target_tag_id  - 要搜索的 Tag ID
          target_distance- 目标距离
          enable_servo   - 是否让视觉伺服控制器接管
          mission_signal - None 或 (mission_id, tag_id, distance)
        """
        if self.state_enter_time is None:
            self.state_enter_time = now

        r = self._empty_result()

        # 根据当前状态分发
        if self.state == self.SEARCH_TAG:
            self._run_search(r, tag, now)
        elif self.state == self.TRACK_TAG:
            self._run_track(r, tag, now)
        elif self.state == self.PREPARE:
            self._run_prepare(r, tag, now)
        elif self.state == self.BLIND_APPROACH:
            self._run_blind(r, now)
        elif self.state == self.STOP:
            self._run_stop(r, now)
        elif self.state == self.SEND_SIGNAL:
            self._run_signal(r, now)
        elif self.state == self.FINISHED:
            self._run_finished(r)

        return r

    def _empty_result(self):
        return {
            'action':          '',
            'cmd_vx':          0.0,
            'cmd_vy':          0.0,
            'cmd_wz':          0.0,
            'target_tag_id':   self.target_tag_id,
            'target_distance': self.target_distance,
            'enable_servo':    False,
            'mission_signal':  None,
        }

    def info(self):
        """返回当前状态摘要 (用于日志)"""
        return {
            'state':          self.state,
            'mission_id':     self.mission_id,
            'target_tag_id':  self.target_tag_id,
            'target_distance': self.target_distance,
            'prepare_dist':   self.prepare_dist,
            'mission_idx':    self.mission_idx,
            'total':          len(self.missions),
        }

    # -------------------------------------------------------------------
    # 状态转移 & Tag 匹配
    # -------------------------------------------------------------------

    def _goto(self, new_state, now):
        old = self.state
        self.state = new_state
        self.state_enter_time = now
        return f'{old} → {new_state}'

    def _tag_ok(self, tag):
        """Tag 存在且 ID 匹配当前任务的 target_tag_id"""
        return tag is not None and tag[3] == self.target_tag_id

    def _save_tag(self, tag, now):
        """保存最近一次有效的 Tag 数据"""
        if tag is not None:
            self.last_tag = tag
            self.last_tag_time = now

    def _tag_lost(self, now):
        """Tag 是否已丢失超过超时时间"""
        if self.last_tag_time is None:
            return True
        return (now - self.last_tag_time) > self.p.get('tag_timeout', 2.0)

    @staticmethod
    def _norm_angle(a):
        """角度归一化到 [-pi, pi]"""
        return math.atan2(math.sin(a), math.cos(a))

    # -------------------------------------------------------------------
    # 各状态的处理函数
    # -------------------------------------------------------------------

    def _run_search(self, r, tag, now):
        """
        SEARCH_TAG: 原地旋转寻找目标 Tag。

        如果所有任务已完成 → FINISHED
        如果检测到目标 Tag  → TRACK_TAG
        否则                 → 继续旋转
        """
        r['enable_servo'] = False

        if self.done:
            r['action'] = self._goto(self.FINISHED, now)
            return

        if self._tag_ok(tag):
            self._save_tag(tag, now)
            r['action'] = self._goto(self.TRACK_TAG, now)
            r['enable_servo'] = True
            return

        # 旋转搜索
        r['cmd_wz'] = self.p.get('search_yaw_rate', 0.3)
        r['action'] = '旋转搜索中...'

    def _run_track(self, r, tag, now):
        """
        TRACK_TAG: 视觉伺服跟踪。

        如果 Tag 丢失超时  → SEARCH_TAG
        如果进入 PREPARE 范围 → PREPARE
        否则                 → 继续跟踪 (伺服控制器负责 cmd_vel)
        """
        r['enable_servo'] = True

        if not self._tag_ok(tag):
            if self._tag_lost(now):
                r['action'] = self._goto(self.SEARCH_TAG, now)
                r['enable_servo'] = False
            return

        self._save_tag(tag, now)
        _, z, _, _ = tag

        if z < self.prepare_dist:
            r['action'] = self._goto(self.PREPARE, now)
        else:
            r['action'] = '跟踪中'

    def _run_prepare(self, r, tag, now):
        """
        PREPARE: 接近目标，随时准备盲走。

        如果到达 stop_distance → STOP
        如果 Tag 丢失          → BLIND_APPROACH
        否则                   → 继续跟踪，同时更新 IMU 航向参考
        """
        r['enable_servo'] = True

        if self._tag_ok(tag):
            self._save_tag(tag, now)
            _, z, _, _ = tag

            # 更新盲走参考航向
            self.blind_yaw_ref = self.imu_yaw

            if z < self.target_distance:
                r['action'] = self._goto(self.STOP, now)
                r['enable_servo'] = False
            else:
                r['action'] = '准备中 (PREPARE)'
        else:
            # Tag 丢了 → 盲走
            if self._tag_lost(now) and self.last_tag is not None:
                self._start_blind(now)
                r['action'] = self._goto(self.BLIND_APPROACH, now)
                r['enable_servo'] = False

    # -------------------------------------------------------------------
    # 盲走
    # -------------------------------------------------------------------

    def _start_blind(self, now):
        """初始化盲走参数"""
        _, last_z, _, _ = self.last_tag
        remain = last_z - self.target_distance
        if remain <= 0.0:
            remain = 0.05           # 最少走 5cm

        blind_vx = self.p.get('blind_vx', 0.1)
        self.blind_travel = remain / abs(blind_vx) if abs(blind_vx) > 0.001 else 5.0
        self.blind_start = now
        self.blind_yaw_ref = self.imu_yaw

    def _run_blind(self, r, now):
        """
        BLIND_APPROACH: 盲走模式。

        恒定 vx 前进，IMU 保持航向。
        时间到或超时 → STOP
        """
        dt = now - self.blind_start
        max_dt = self.p.get('blind_approach_timeout', 10.0)

        if dt >= max_dt:
            r['action'] = self._goto(self.STOP, now) + ' (盲走超时)'
            return

        if dt >= self.blind_travel:
            r['action'] = self._goto(self.STOP, now) + f' (盲走完成, {dt:.1f}s)'
            return

        # 航向修正
        blind_vx = self.p.get('blind_vx', 0.1)
        kp = self.p.get('kp_yaw', 0.5)
        max_w = self.p.get('max_angular_vel', 0.5)

        e_yaw = self._norm_angle(self.blind_yaw_ref - self.imu_yaw)
        wz = max(-max_w, min(max_w, kp * e_yaw))

        r['cmd_vx'] = blind_vx
        r['cmd_wz'] = wz
        r['action'] = f'盲走 {dt:.1f}s / {self.blind_travel:.1f}s'

    # -------------------------------------------------------------------
    # 停止 & 发信号
    # -------------------------------------------------------------------

    def _run_stop(self, r, now):
        """STOP: 短暂保持静止，然后发信号"""
        r['cmd_vx'] = 0.0
        r['cmd_vy'] = 0.0
        r['cmd_wz'] = 0.0

        if self.stop_start is None:
            self.stop_start = now

        if (now - self.stop_start) >= 0.3:               # 停稳 0.3 秒
            self.signal_start = now
            r['action'] = self._goto(self.SEND_SIGNAL, now)
        else:
            r['action'] = '短暂停止中'

    def _run_signal(self, r, now):
        """SEND_SIGNAL: 发布任务完成信号"""
        r['cmd_vx'] = 0.0
        r['cmd_vy'] = 0.0
        r['cmd_wz'] = 0.0

        # 实际距离优先用最后看到的 Tag
        dist = self.target_distance
        if self.last_tag is not None:
            dist = float(self.last_tag[1])

        r['mission_signal'] = (self.mission_id, self.target_tag_id, dist)

        duration = self.p.get('signal_duration', 1.0)
        if (now - self.signal_start) >= duration:
            r['action'] = self._goto(self.FINISHED, now)
            self._next_mission()       # 加载下一个任务
        else:
            r['action'] = '发送信号中'

    def _run_finished(self, r):
        """FINISHED: 检查是否还有下一个任务"""
        r['cmd_vx'] = 0.0
        r['cmd_vy'] = 0.0
        r['cmd_wz'] = 0.0

        if not self.done:
            # 还有任务 → 重置，回到 SEARCH
            self.state = self.SEARCH_TAG
            self.state_enter_time = None
            self.last_tag = None
            self.last_tag_time = None
            self.blind_start = None
            self.stop_start = None
            self.signal_start = None
            r['action'] = f'开始下一任务 (mission_id={self.mission_id})'
        else:
            r['action'] = '全部任务完成，等待指令'

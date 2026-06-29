"""
任务状态机 —— 纯逻辑，不依赖 ROS。

状态流转 (支持多步骤任务):
  SEARCH_TAG → TRACK_TAG → PREPARE → BLIND_APPROACH → STOP
                                                         |
                              ┌──────────────────────────┤
                              │                          │
                         (有下一步骤)                (无下一步骤)
                              │                          │
              TURN / CORRECT_HEADING / DRIVE_FORWARD   FINISHED
                              │                          │
                         (还有步骤?)                (有下个mission)
                         ╱         ╲                    │
                      是            否              SEARCH_TAG
                      │             │
                  SEND_SIGNAL    SEND_SIGNAL
                      │             │
                  FINISHED       FINISHED
                      │             │
                  _next_step    _next_mission

MissionManagerNode (LifecycleNode) 包装这个类，负责所有 ROS 通信。
"""

import math
from enum import IntEnum


# ---------------------------------------------------------------------------
# ObstacleType — 障碍物类型枚举 (外部控制板根据 mission_id 自行查表)
# ---------------------------------------------------------------------------

class ObstacleType(IntEnum):
    DEFAULT_WALK   = 0
    CRAWLING_FRAME = 1
    HIGH_WALL      = 2
    STAIR          = 3
    SANDPIT        = 4
    SLOPE          = 5


# ---------------------------------------------------------------------------
# 状态机
# ---------------------------------------------------------------------------

class MissionStateMachine:
    """纯状态机：根据输入数据评估状态转移，返回动作指令"""

    # -- 基础状态 --
    SEARCH_TAG      = "SEARCH_TAG"
    TRACK_TAG       = "TRACK_TAG"
    PREPARE         = "PREPARE"
    BLIND_APPROACH  = "BLIND_APPROACH"
    STOP            = "STOP"
    SEND_SIGNAL     = "SEND_SIGNAL"
    FINISHED        = "FINISHED"

    # -- 新增状态 --
    TURN            = "TURN"
    CORRECT_HEADING = "CORRECT_HEADING"
    DRIVE_FORWARD   = "DRIVE_FORWARD"

    # -- 步骤类型 --
    STEP_APPROACH        = "approach"
    STEP_TURN            = "turn"
    STEP_SIGNAL          = "signal"
    STEP_FORWARD         = "forward"
    STEP_CORRECT_HEADING = "correct_heading"

    def __init__(self, missions, params):
        """
        missions: 任务列表，每项至少 {'mission_id', 'tag_id', 'stop_distance'}
                  可选 'steps' 列表，每项 {'type': ..., ...}
        params:   状态机参数 (见 mission_params.yaml)
        """
        self.missions = missions
        self.mission_idx = 0
        self.p = params

        # 当前状态
        self.state = self.SEARCH_TAG

        # 当前任务目标
        self.target_tag_id = -1
        self.target_x = 0.0
        self.target_z = 0.5
        self.mission_id = -1

        # 步骤定序器
        self._steps = []              # 当前 mission 的步骤列表
        self._step_idx = 0            # 当前步骤索引
        self._current_step_type = None

        # 步骤参数 (由 _load_step 设置)
        self.turn_angle = 0.0         # 转向角度 (deg, 正=右转)
        self.forward_distance = 1.0   # 盲走前进距离 (m)
        self.correction_tag_id = -1   # 航向修正用的 Tag ID

        self._load_mission()

        # 跟踪数据
        self.last_tag = None
        self.last_tag_time = None
        self.imu_yaw = 0.0

        # IMU 滤波
        self._yaw_buffer = []
        self._yaw_buffer_max = int(self.p.get('imu_filter_window', 20))

        # 盲走参数
        self.blind_yaw_ref = 0.0
        self.blind_travel = 0.0
        self.blind_start = None

        # STOP / SIGNAL 计时
        self.stop_start = None
        self.signal_start = None

        # 转向参数 (在 _start_turn 中设置)
        self.turn_start_yaw = 0.0
        self.turn_target_yaw = 0.0
        self.turn_start_time = None
        self._turn_done = False

        # 盲走前进参数 (在 _start_forward 中设置)
        self.forward_start_time = None
        self.forward_ref_yaw = 0.0
        self.forward_travel = 0.0

        # 航向修正参数
        self._correction_start_time = None
        self._correction_tag_found = False

        # 下一步状态路由 (STOP 之后去哪)
        self._next_state_after_stop = None

        # 信号发送开关 (步骤中遇到 signal 类型才发)
        self._send_signal = False

        # 状态进入时间
        self.state_enter_time = None

    # -------------------------------------------------------------------
    # 属性
    # -------------------------------------------------------------------

    @property
    def prepare_dist(self):
        """进入 PREPARE 的距离阈值 = 到目标点的距离 + offset"""
        return self.target_distance + self.p.get('prepare_distance_offset', 0.3)

    @property
    def done(self):
        """所有任务是否已完成"""
        return self.mission_idx >= len(self.missions)

    # -------------------------------------------------------------------
    # 任务 & 步骤管理
    # -------------------------------------------------------------------

    def _load_mission(self):
        """从队列加载当前任务及其步骤"""
        if self.done:
            self.target_tag_id = -1
            self.target_x = 0.0
            self.target_z = 0.5
            self.mission_id = -1
            self._steps = []
            self._step_idx = 0
            self._current_step_type = None
            return
        m = self.missions[self.mission_idx]
        self.target_tag_id = m['tag_id']
        self.target_z = float(m.get('target_z', m.get('stop_distance', 0.5)))
        self.target_x = float(m.get('target_x', 0.0))
        self.mission_id = m.get('mission_id', self.mission_idx)

        # 向后兼容: 无 steps → 默认 [{approach}, {signal}]
        raw_steps = m.get('steps', None)
        if raw_steps is None:
            self._steps = [{'type': self.STEP_APPROACH}, {'type': self.STEP_SIGNAL}]
        else:
            self._steps = list(raw_steps)

        self._step_idx = 0
        self._load_step()

    def _load_step(self):
        """将当前步骤的参数加载到状态变量，并决定进入哪个状态"""
        if self._step_idx >= len(self._steps):
            self._current_step_type = None
            return

        step = self._steps[self._step_idx]
        self._current_step_type = step.get('type', self.STEP_APPROACH)

        if self._current_step_type == self.STEP_TURN:
            self.turn_angle = float(step.get('angle', 0.0))
        elif self._current_step_type == self.STEP_FORWARD:
            self.forward_distance = float(step.get('distance', 1.0))
        elif self._current_step_type == self.STEP_CORRECT_HEADING:
            self.correction_tag_id = int(step.get('tag_id', self.target_tag_id))

    def _next_step(self):
        """推进到下一步骤；如果全部完成 → 加载下一个 mission"""
        self._step_idx += 1
        if self._step_idx >= len(self._steps):
            self._next_mission()
        else:
            self._load_step()

    def _next_mission(self):
        """加载下一个任务"""
        self.mission_idx += 1
        self._load_mission()

    @property
    def target_distance(self):
        """到目标点的欧氏距离 = sqrt(target_x² + target_z²)"""
        return math.sqrt(self.target_x ** 2 + self.target_z ** 2)

    # -------------------------------------------------------------------
    # 外部接口
    # -------------------------------------------------------------------

    def update_imu(self, yaw):
        """更新 IMU 航向 (由节点回调调用)，维护滑动窗口"""
        self.imu_yaw = yaw
        self._yaw_buffer.append(yaw)
        if len(self._yaw_buffer) > self._yaw_buffer_max:
            self._yaw_buffer.pop(0)

    def _filtered_yaw(self):
        """IMU 航向圆形均值滤波"""
        if not self._yaw_buffer:
            return self.imu_yaw
        s = sum(math.sin(a) for a in self._yaw_buffer)
        c = sum(math.cos(a) for a in self._yaw_buffer)
        return math.atan2(s, c)

    def evaluate(self, tag, now):
        """
        每个控制周期调用一次。

        tag: (x, z, yaw, id) 或 None
        now: 当前时间 (秒, time.monotonic())

        返回 dict: action, cmd_vx, cmd_vy, cmd_wz, target_tag_id,
                   target_x, target_z, target_distance, enable_servo,
                   mission_signal
        """
        if self.state_enter_time is None:
            self.state_enter_time = now

        r = self._empty_result()

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
            self._run_finished(r, now)
        elif self.state == self.TURN:
            self._run_turn(r, now)
        elif self.state == self.CORRECT_HEADING:
            self._run_correct_heading(r, tag, now)
        elif self.state == self.DRIVE_FORWARD:
            self._run_forward(r, now)

        return r

    def _empty_result(self):
        return {
            'action':          '',
            'cmd_vx':          0.0,
            'cmd_vy':          0.0,
            'cmd_wz':          0.0,
            'target_tag_id':   self.target_tag_id,
            'target_x':        self.target_x,
            'target_z':        self.target_z,
            'target_distance': self.target_distance,
            'enable_servo':    False,
            'mission_signal':  None,
            'turn_angle':      self.turn_angle,
            'forward_distance': self.forward_distance,
            'step_idx':        self._step_idx,
            'step_total':      len(self._steps),
        }

    def info(self):
        """返回当前状态摘要 (用于日志)"""
        return {
            'state':           self.state,
            'mission_id':      self.mission_id,
            'target_tag_id':   self.target_tag_id,
            'target_x':        self.target_x,
            'target_z':        self.target_z,
            'target_distance': self.target_distance,
            'prepare_dist':    self.prepare_dist,
            'mission_idx':     self.mission_idx,
            'total':           len(self.missions),
            'step_idx':        self._step_idx,
            'step_total':      len(self._steps),
            'step_type':       self._current_step_type,
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
        return tag is not None and tag[3] == self.target_tag_id

    def _dist_to_target(self, tag):
        tx, tz, _, _ = tag
        dx = tx + self.target_x
        dz = tz - self.target_z
        return math.sqrt(dx * dx + dz * dz)

    def _save_tag(self, tag, now):
        if tag is not None:
            self.last_tag = tag
            self.last_tag_time = now

    def _tag_lost(self, now):
        if self.last_tag_time is None:
            return True
        return (now - self.last_tag_time) > self.p.get('tag_timeout', 2.0)

    @staticmethod
    def _norm_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    # -------------------------------------------------------------------
    # SEARCH_TAG
    # -------------------------------------------------------------------

    def _run_search(self, r, tag, now):
        """
        SEARCH_TAG: 原地旋转寻找目标 Tag。

        如果首步骤不是 approach (即 turn/forward/correct_heading):
          跳过 SEARCH 直接进入对应状态。
        如果已检测到目标 Tag → TRACK_TAG
        否则 → 继续旋转
        """
        r['enable_servo'] = False

        if self.done:
            r['action'] = self._goto(self.FINISHED, now)
            return

        # 如果当前步骤不是 approach，直接跳转到对应状态
        if self._current_step_type == self.STEP_TURN:
            self._start_turn(now)
            r['action'] = self._goto(self.TURN, now)
            return
        elif self._current_step_type == self.STEP_FORWARD:
            self._start_forward(now)
            r['action'] = self._goto(self.DRIVE_FORWARD, now)
            return
        elif self._current_step_type == self.STEP_CORRECT_HEADING:
            self._start_correction(now)
            r['action'] = self._goto(self.CORRECT_HEADING, now)
            return
        elif self._current_step_type == self.STEP_SIGNAL:
            # 纯发信号步骤 (无 approach)
            self.signal_start = now
            self._send_signal = True
            r['action'] = self._goto(self.SEND_SIGNAL, now)
            return

        # 正常 approach 流程
        if self._tag_ok(tag):
            self._save_tag(tag, now)
            # 对准了再进 TRACK，避免边缘抓到 Tag 就冲
            align_thresh = self.p.get('search_align_thresh', 0.15)
            if abs(tag[0]) < align_thresh:
                r['action'] = self._goto(self.TRACK_TAG, now)
                r['enable_servo'] = True
                return
            # Tag 偏了 → P 控制旋转对中再进 TRACK
            kp = self.p.get('search_align_kp', 0.4)
            max_w = self.p.get('max_angular_vel', 0.5)
            r['cmd_wz'] = max(-max_w, min(max_w, -kp * tag[0]))
            r['action'] = f'对准中... x={tag[0]:.2f}m'
            return

        r['cmd_wz'] = self.p.get('search_yaw_rate', 0.3)
        r['action'] = '旋转搜索中...'

    # -------------------------------------------------------------------
    # TRACK_TAG
    # -------------------------------------------------------------------

    def _run_track(self, r, tag, now):
        r['enable_servo'] = True

        if not self._tag_ok(tag):
            if self._tag_lost(now):
                r['action'] = self._goto(self.SEARCH_TAG, now)
                r['enable_servo'] = False
            return

        self._save_tag(tag, now)

        if self._dist_to_target(tag) < self.prepare_dist:
            r['action'] = self._goto(self.PREPARE, now)
        else:
            r['action'] = '跟踪中'

    # -------------------------------------------------------------------
    # PREPARE
    # -------------------------------------------------------------------

    def _run_prepare(self, r, tag, now):
        r['enable_servo'] = True

        if self._tag_ok(tag):
            self._save_tag(tag, now)
            self.blind_yaw_ref = self.imu_yaw

            if self._dist_to_target(tag) < 0.03:
                # 到达目标点 → STOP
                r['action'] = self._goto(self.STOP, now)
                r['enable_servo'] = False
            else:
                r['action'] = '准备中 (PREPARE)'
        else:
            if self._tag_lost(now) and self.last_tag is not None:
                self._start_blind(now)
                r['action'] = self._goto(self.BLIND_APPROACH, now)
                r['enable_servo'] = False

    # -------------------------------------------------------------------
    # 盲走
    # -------------------------------------------------------------------

    def _start_blind(self, now):
        remain = self._dist_to_target(self.last_tag)
        if remain <= 0.0:
            remain = 0.05

        blind_vx = self.p.get('blind_vx', 0.1)
        self.blind_travel = remain / abs(blind_vx) if abs(blind_vx) > 0.001 else 5.0
        self.blind_start = now
        self.blind_yaw_ref = self.imu_yaw

    def _run_blind(self, r, now):
        dt = now - self.blind_start
        max_dt = self.p.get('blind_approach_timeout', 10.0)

        if dt >= max_dt:
            r['action'] = self._goto(self.STOP, now) + ' (盲走超时)'
            return

        if dt >= self.blind_travel:
            r['action'] = self._goto(self.STOP, now) + f' (盲走完成, {dt:.1f}s)'
            return

        blind_vx = self.p.get('blind_vx', 0.1)
        kp = self.p.get('kp_yaw', 0.5)
        max_w = self.p.get('max_angular_vel', 0.5)

        e_yaw = self._norm_angle(self.blind_yaw_ref - self.imu_yaw)
        wz = max(-max_w, min(max_w, kp * e_yaw))

        r['cmd_vx'] = blind_vx
        r['cmd_wz'] = wz
        r['action'] = f'盲走 {dt:.1f}s / {self.blind_travel:.1f}s'

    # -------------------------------------------------------------------
    # STOP — 根据下一步骤决定分发目标
    # -------------------------------------------------------------------

    def _run_stop(self, r, now):
        """
        STOP: approach 步骤的终点。

        停稳 0.3s 后推进到下一步骤，根据新步骤类型路由:
          turn → TURN / correct_heading → CORRECT_HEADING
          forward → DRIVE_FORWARD / signal → SEND_SIGNAL
          无更多步骤 → FINISHED
        """
        r['cmd_vx'] = 0.0
        r['cmd_vy'] = 0.0
        r['cmd_wz'] = 0.0

        if self.stop_start is None:
            self.stop_start = now

        if (now - self.stop_start) >= 0.3:
            # 推进到下一步骤
            self._next_step()

            if self.done:
                # 所有 mission 完成
                r['action'] = self._goto(self.FINISHED, now)
                return

            # 根据新步骤类型路由
            st = self._current_step_type
            if st == self.STEP_TURN:
                self._start_turn(now)
                r['action'] = self._goto(self.TURN, now)
            elif st == self.STEP_CORRECT_HEADING:
                self._start_correction(now)
                r['action'] = self._goto(self.CORRECT_HEADING, now)
            elif st == self.STEP_FORWARD:
                self._start_forward(now)
                r['action'] = self._goto(self.DRIVE_FORWARD, now)
            elif st == self.STEP_SIGNAL:
                self.signal_start = now
                self._send_signal = True
                r['action'] = self._goto(self.SEND_SIGNAL, now)
            else:
                # 未知类型 → FINISHED
                r['action'] = self._goto(self.FINISHED, now)
        else:
            r['action'] = '短暂停止中'

    # -------------------------------------------------------------------
    # SEND_SIGNAL
    # -------------------------------------------------------------------

    def _run_signal(self, r, now):
        r['cmd_vx'] = 0.0
        r['cmd_vy'] = 0.0
        r['cmd_wz'] = 0.0

        dist = self.target_distance
        if self.last_tag is not None:
            dist = float(self.last_tag[1])

        r['mission_signal'] = (
            self.mission_id,
            self.target_tag_id,
            dist,
            self._filtered_yaw(),
        )

        duration = self.p.get('signal_duration', 1.0)
        if (now - self.signal_start) >= duration:
            r['action'] = self._goto(self.FINISHED, now)
        else:
            r['action'] = '发送信号中'

    # -------------------------------------------------------------------
    # FINISHED
    # -------------------------------------------------------------------

    def _run_finished(self, r, now):
        """
        FINISHED: 推进到下一步骤/下一个 mission，并路由到正确的起始状态。

        每个步骤完成后的统一入口。在此处:
          1. 推进 _step_idx (调用 _next_step)
          2. 重置状态变量
          3. 根据新的步骤类型路由到对应状态
        """
        r['cmd_vx'] = 0.0
        r['cmd_vy'] = 0.0
        r['cmd_wz'] = 0.0

        if self.done:
            r['action'] = '全部任务完成，等待指令'
            return

        # 推进步骤 (当前步骤已在前一个状态中执行完毕)
        self._next_step()

        if self.done:
            r['action'] = '全部任务完成，等待指令'
            return

        # 重置所有步骤级状态
        self.state_enter_time = None
        self.last_tag = None
        self.last_tag_time = None
        self.blind_start = None
        self.stop_start = None
        self.signal_start = None
        self.turn_start_time = None
        self.forward_start_time = None
        self._correction_start_time = None
        self._correction_tag_found = False
        self._send_signal = False

        # 根据新步骤类型路由
        st = self._current_step_type
        if st == self.STEP_APPROACH:
            self.state = self.SEARCH_TAG
            r['action'] = f'→ 下一步: approach (tag_id={self.target_tag_id})'
        elif st == self.STEP_TURN:
            self._start_turn(now)
            self.state = self.TURN
            r['action'] = f'→ 下一步: turn {self.turn_angle:.0f}°'
        elif st == self.STEP_CORRECT_HEADING:
            self._start_correction(now)
            self.state = self.CORRECT_HEADING
            r['action'] = f'→ 下一步: correct_heading (tag_id={self.correction_tag_id})'
        elif st == self.STEP_FORWARD:
            self._start_forward(now)
            self.state = self.DRIVE_FORWARD
            r['action'] = f'→ 下一步: forward {self.forward_distance:.1f}m'
        elif st == self.STEP_SIGNAL:
            self.signal_start = now
            self._send_signal = True
            self.state = self.SEND_SIGNAL
            r['action'] = '→ 下一步: signal'
        else:
            self.state = self.SEARCH_TAG
            r['action'] = f'→ 下一步: 未知类型 "{st}", 回退到 SEARCH'

    # ===================================================================
    # TURN — IMU 精确转向
    # ===================================================================

    def _start_turn(self, now):
        """记录滤波后的起始航向，计算目标绝对航向"""
        self.turn_start_yaw = self._filtered_yaw()
        target_rad = math.radians(self.turn_angle)
        self.turn_target_yaw = self._norm_angle(self.turn_start_yaw + target_rad)
        self.turn_start_time = now
        self._turn_done = False

    def _run_turn(self, r, now):
        """TURN: 原地旋转到目标 IMU 航向"""
        turn_rate = self.p.get('turn_yaw_rate', 0.5)
        tolerance = self.p.get('turn_tolerance', 0.05)
        timeout = self.p.get('turn_timeout', 15.0)
        settle = self.p.get('turn_settle_time', 0.3)
        kp = self.p.get('turn_kp_yaw', 1.0)

        # settle 期 — 让 IMU 缓冲填满
        elapsed = now - self.turn_start_time
        if elapsed < settle:
            r['cmd_vx'] = 0.0
            r['cmd_wz'] = 0.0
            r['action'] = f'TURN settle {(elapsed):.1f}s'
            return

        # 计算误差
        current_yaw = self._filtered_yaw()
        error = self._norm_angle(self.turn_target_yaw - current_yaw)

        # 完成判定
        if abs(error) < tolerance:
            self._turn_done = True
            r['action'] = (f'转向完成 {self.turn_angle:.0f}° '
                           f'(err={math.degrees(error):.1f}°)')
            r['action'] += ' → ' + self._goto(self.FINISHED, now)
            return

        if elapsed > timeout:
            self._turn_done = True
            r['action'] = f'转向超时 (err={math.degrees(error):.1f}°)'
            r['action'] += ' → ' + self._goto(self.FINISHED, now)
            return

        # P 控制角速度
        wz = max(-turn_rate, min(turn_rate, kp * error))
        r['cmd_vx'] = 0.0
        r['cmd_wz'] = wz
        r['action'] = (f'转向中 err={math.degrees(error):.1f}° '
                       f'/ {self.turn_angle:.0f}°')


    # ===================================================================
    # CORRECT_HEADING — AprilTag 航向修正
    # ===================================================================

    def _start_correction(self, now):
        """初始化航向修正"""
        self._correction_start_time = now
        self._correction_tag_found = False

    def _run_correct_heading(self, r, tag, now):
        """
        CORRECT_HEADING: 旋转使机器人正对 AprilTag。
        搜索 correction_tag_id → 修正 tag_yaw → 0
        """
        correction_rate = self.p.get('correction_yaw_rate', 0.3)
        tolerance = self.p.get('correction_tolerance', 0.04)
        timeout = self.p.get('correction_timeout', 10.0)
        kp = self.p.get('correction_kp_yaw', 1.0)

        elapsed = now - self._correction_start_time

        if elapsed > timeout:
            r['action'] = f'航向修正超时 ({self.correction_tag_id})'
            r['action'] += ' → ' + self._goto(self.FINISHED, now)
            return

        # 检查是否检测到修正 Tag
        correction_tag = None
        if tag is not None and tag[3] == self.correction_tag_id:
            correction_tag = tag
            self._correction_tag_found = True
            self._save_tag(tag, now)

        if correction_tag is None:
            # 旋转搜索修正 Tag
            r['cmd_wz'] = correction_rate * 0.5
            r['cmd_vx'] = 0.0
            if self._correction_tag_found:
                r['action'] = f'修正Tag丢失，重新搜索 (id={self.correction_tag_id})'
            else:
                r['action'] = f'搜索修正 Tag (id={self.correction_tag_id})'
            return

        # Tag 找到 → 用 tag_yaw 修正航向
        tag_yaw = correction_tag[2]
        if abs(tag_yaw) < tolerance:
            r['action'] = (f'航向修正好 (id={self.correction_tag_id}, '
                           f'err={math.degrees(tag_yaw):.1f}°)')
            r['action'] += ' → ' + self._goto(self.FINISHED, now)
            return

        wz = max(-correction_rate, min(correction_rate, kp * tag_yaw))
        r['cmd_vx'] = 0.0
        r['cmd_wz'] = wz
        r['action'] = f'修正航向 err={math.degrees(tag_yaw):.1f}°'

    # ===================================================================
    # DRIVE_FORWARD — 盲走前进
    # ===================================================================

    def _start_forward(self, now):
        """初始化盲走前进"""
        forward_vx = self.p.get('forward_vx', 0.15)
        self.forward_travel = (self.forward_distance /
                               abs(forward_vx) if abs(forward_vx) > 0.001 else 5.0)
        self.forward_start_time = now
        self.forward_ref_yaw = self._filtered_yaw()

    def _run_forward(self, r, now):
        """
        DRIVE_FORWARD: 恒定 vx 前进，IMU 保持航向。
        """
        dt = now - self.forward_start_time
        timeout = self.p.get('forward_timeout', 30.0)
        forward_vx = self.p.get('forward_vx', 0.15)
        kp = self.p.get('kp_yaw', 0.5)
        max_w = self.p.get('max_angular_vel', 0.5)

        if dt >= timeout:
            r['action'] = (f'盲走前进超时 ({self.forward_distance:.1f}m)')
            r['action'] += ' → ' + self._goto(self.FINISHED, now)
            return

        if dt >= self.forward_travel:
            r['action'] = (f'盲走前进完成 {self.forward_distance:.1f}m '
                           f'({dt:.1f}s)')
            r['action'] += ' → ' + self._goto(self.FINISHED, now)
            return

        # 航向保持
        e_yaw = self._norm_angle(self.forward_ref_yaw - self.imu_yaw)
        wz = max(-max_w, min(max_w, kp * e_yaw))

        r['cmd_vx'] = forward_vx
        r['cmd_wz'] = wz
        r['action'] = (f'盲走前进 {dt:.1f}s / {self.forward_travel:.1f}s '
                       f'({dt * forward_vx:.2f}m / {self.forward_distance:.1f}m)')

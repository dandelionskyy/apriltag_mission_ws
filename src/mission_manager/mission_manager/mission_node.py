#!/usr/bin/env python3
"""
任务管理器节点 (LifecycleNode)。

统一调度整个任务流程:
  1. 用参数服务远程控制检测器 (target_tag_id) 和伺服控制器 (enable, target_distance)
  2. 以固定频率运行状态机
  3. 在 SEARCH 和 BLIND_APPROACH 状态直接发 /cmd_vel
  4. 任务完成时发布 /mission_signal

LifecycleNode 生命周期:
  on_configure() → 加载配置、创建状态机、发布/订阅、参数客户端
  on_activate()  → 启动控制定时器
  on_deactivate()→ 停车、关闭定时器
  on_cleanup()   → 销毁资源
"""

import math
import time
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from apriltag_interfaces.msg import TagPose
from apriltag_interfaces.srv import TriggerMission
from .state_machine import MissionStateMachine

try:
    from scipy.spatial.transform import Rotation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------

def _yaw_from_quat(x, y, z, w):
    """四元数 → yaw (rad)"""
    if HAS_SCIPY:
        r = Rotation.from_quat([x, y, z, w])
        _, _, yaw = r.as_euler('xyz', degrees=False)
        return yaw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _make_param_msg(name, value):
    """构建 rcl_interfaces/Parameter 消息，用于远程设参"""
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, int):
        pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, value
    elif isinstance(value, float):
        pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, value
    elif isinstance(value, str):
        pv.type, pv.string_value = ParameterType.PARAMETER_STRING, value
    else:
        return None
    pm = ParamMsg(name=name, value=pv)
    return pm


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------

class MissionManagerNode(LifecycleNode):

    def __init__(self):
        super().__init__('mission_manager')

        self.sm = None                     # 状态机
        self._timer = None                 # 控制循环定时器

        # 缓存的最新数据
        self._tag = None                   # (x, z, yaw, id) 或 None
        self._tag_new = False
        self._imu_yaw = 0.0
        self._last_imu_time = None        # 用于角速度积分
        self._imu_has_orientation = True  # IMU 是否提供融合后的姿态
        self._last_loop_time = None       # 用于 cmd_wz 积分 (Livox无orientation时)
        self._last_cmd_wz = 0.0           # 上一周期发出的角速度

        # 发布者 & 订阅者 & 服务客户端
        self._pub_cmd = None
        self._cli_trigger = None     # 调用外部控制板的 TriggerMission 服务
        self._sub_tag = None
        self._sub_imu = None

        # 服务调用状态 (避免每个循环周期都调)
        self._pending_signal = None   # 待发送的 (mission_id, tag_id, distance, angle)
        self._signal_sent = False     # 本段信号是否已发送成功

        # 远程参数客户端
        self._cli_detector = None          # → apriltag_detector/set_parameters
        self._cli_servo    = None          # → visual_servo_controller/set_parameters

        # 缓存上次设置的值，避免重复设参
        self._cache_tag_id = None
        self._cache_enable = None
        self._cache_dist   = None
        self._cache_x      = None

        # 日志去重
        self._last_state = None
        self._last_action = None

    # ===================================================================
    # LifecycleNode 回调
    # ===================================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() — 加载配置...')

        # -- 声明 & 读取参数 --
        self._declare_params()
        missions = self._load_missions()
        if not missions:
            self.get_logger().error('missions 为空！')
            return TransitionCallbackReturn.ERROR

        params = {
            'prepare_distance_offset': self.get_parameter('prepare_distance_offset').value,
            'tag_timeout':             self.get_parameter('tag_timeout').value,
            'search_yaw_rate':         self.get_parameter('search_yaw_rate').value,
            'blind_vx':               self.get_parameter('blind_vx').value,
            'blind_approach_timeout':  self.get_parameter('blind_approach_timeout').value,
            'kp_yaw':                 self.get_parameter('kp_yaw').value,
            'max_angular_vel':         self.get_parameter('max_angular_vel').value,
            'signal_duration':         self.get_parameter('signal_duration').value,
            'loop_rate':              self.get_parameter('loop_rate').value,
            # 转向
            'turn_yaw_rate':           self.get_parameter('turn_yaw_rate').value,
            'turn_tolerance':          self.get_parameter('turn_tolerance').value,
            'turn_timeout':            self.get_parameter('turn_timeout').value,
            'turn_settle_time':        self.get_parameter('turn_settle_time').value,
            'turn_kp_yaw':             self.get_parameter('turn_kp_yaw').value,
            # 航向修正
            'correction_yaw_rate':     self.get_parameter('correction_yaw_rate').value,
            'correction_tolerance':    self.get_parameter('correction_tolerance').value,
            'correction_timeout':      self.get_parameter('correction_timeout').value,
            'correction_kp_yaw':       self.get_parameter('correction_kp_yaw').value,
            # 盲走前进
            'forward_vx':              self.get_parameter('forward_vx').value,
            'forward_timeout':         self.get_parameter('forward_timeout').value,
            # IMU 滤波
            'search_align_thresh':     self.get_parameter('search_align_thresh').value,
            'search_align_kp':         self.get_parameter('search_align_kp').value,
            'imu_filter_window':       self.get_parameter('imu_filter_window').value,
        }

        # -- 创建状态机 --
        self.sm = MissionStateMachine(missions, params)
        m0 = missions[0]
        self.get_logger().info(
            f'已加载 {len(missions)} 个任务。'
            f'首任务: mission_id={m0["mission_id"]}, '
            f'tag_id={m0["tag_id"]}, stop={m0["stop_distance"]}m'
        )

        # -- 发布者 --
        self._pub_cmd = self.create_lifecycle_publisher(Twist, '/cmd_vel_nav', 10)

        # -- 服务客户端 (调用外部控制板) --
        self._cli_trigger = self.create_client(
            TriggerMission, '/mission_signal'
        )

        # -- 订阅者 --
        self._sub_tag = self.create_subscription(
            TagPose, '/tag_pose', self._cb_tag, 10
        )
        self._sub_imu = self.create_subscription(
            Imu, '/livox/imu', self._cb_imu, 10
        )

        # -- 远程参数客户端 --
        self._cli_detector = self.create_client(
            SetParameters, '/apriltag_detector/set_parameters'
        )
        self._cli_servo = self.create_client(
            SetParameters, '/visual_servo_controller/set_parameters'
        )

        self.get_logger().info('on_configure() 完成')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate() — 启动控制循环')

        # 重置状态机
        self.sm.state = MissionStateMachine.SEARCH_TAG
        self.sm.state_enter_time = None
        self.sm.last_tag = None
        self.sm.last_tag_time = None

        # 清远程参数缓存（强制重新推送）
        self._cache_tag_id = None
        self._cache_enable = None
        self._cache_dist = None
        self._cache_x = None
        self._pending_signal = None
        self._signal_sent = False
        self._imu_has_orientation = True
        self._last_cmd_wz = 0.0
        self._last_loop_time = None

        hz = self.sm.p.get('loop_rate', 50.0)
        self._timer = self.create_timer(1.0 / hz, self._loop)

        self.get_logger().info('on_activate() 完成')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate() — 停止')

        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None

        self._stop_robot()
        self._set_remote(self._cli_servo, [('enable', False)])

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup()')
        self.sm = None
        self._cli_detector = None
        self._cli_servo = None
        self._cli_trigger = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._stop_robot()
        return TransitionCallbackReturn.SUCCESS

    # ===================================================================
    # 参数加载
    # ===================================================================

    def _declare_params(self):
        # missions 不走 ROS2 参数系统（嵌套结构不支持 --params-file），
        # 改用 missions_file 传路径，on_configure 时直接用 yaml.safe_load 读
        self.declare_parameter('missions_file', '')
        self.declare_parameter('prepare_distance_offset', 0.3)
        self.declare_parameter('tag_timeout', 2.0)
        self.declare_parameter('search_yaw_rate', 0.3)
        self.declare_parameter('blind_vx', 0.1)
        self.declare_parameter('blind_approach_timeout', 10.0)
        self.declare_parameter('kp_yaw', 0.5)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('signal_duration', 1.0)
        self.declare_parameter('loop_rate', 50.0)
        # 转向参数
        self.declare_parameter('turn_yaw_rate', 0.5)
        self.declare_parameter('turn_tolerance', 0.05)
        self.declare_parameter('turn_timeout', 15.0)
        self.declare_parameter('turn_settle_time', 0.3)
        self.declare_parameter('turn_kp_yaw', 1.0)
        # 航向修正参数
        self.declare_parameter('correction_yaw_rate', 0.3)
        self.declare_parameter('correction_tolerance', 0.04)
        self.declare_parameter('correction_timeout', 10.0)
        self.declare_parameter('correction_kp_yaw', 1.0)
        # 盲走前进参数
        self.declare_parameter('forward_vx', 0.15)
        self.declare_parameter('forward_timeout', 30.0)
        # 搜索对准
        self.declare_parameter('search_align_thresh', 0.15)
        self.declare_parameter('search_align_kp', 0.4)
        # IMU 滤波
        self.declare_parameter('imu_filter_window', 20)

    def _load_missions(self):
        """直接从 YAML 文件加载任务列表，绕过 ROS2 参数系统"""
        import yaml, os
        path = self.get_parameter('missions_file').value
        if not path or not os.path.exists(path):
            self.get_logger().error(f'missions_file 不存在: {path}')
            return []

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # 兼容两种格式: 带 ros__parameters 包装的 和 裸列表的
        raw = None
        if isinstance(data, dict):
            # 尝试穿透 /** / ros__parameters / missions
            for node_key in data:
                node_data = data[node_key]
                if isinstance(node_data, dict) and 'ros__parameters' in node_data:
                    raw = node_data['ros__parameters'].get('missions')
                    break
        if raw is None and isinstance(data, list):
            raw = data
        if raw is None:
            raw = data.get('missions', []) if isinstance(data, dict) else []

        result = []
        for m in raw:
            if isinstance(m, dict):
                entry = {
                    'mission_id':    int(m.get('mission_id', 0)),
                    'tag_id':        int(m.get('tag_id', 0)),
                    'stop_distance': float(m.get('stop_distance', 0.5)),
                }
                # 可选字段: target_x, target_z
                if 'target_x' in m:
                    entry['target_x'] = float(m['target_x'])
                if 'target_z' in m:
                    entry['target_z'] = float(m['target_z'])
                # 步骤列表 (向后兼容: 无 steps → 默认 [{approach}, {signal}])
                if 'steps' in m:
                    entry['steps'] = m['steps']
                result.append(entry)
        return result

    # ===================================================================
    # 订阅回调
    # ===================================================================

    def _cb_tag(self, msg: TagPose):
        tag_yaw = _yaw_from_quat(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        self._tag = (msg.pose.position.x, msg.pose.position.z, tag_yaw, msg.tag_id)
        self._tag_new = True

    def _cb_imu(self, msg: Imu):
        # 优先使用 orientation (融合后的姿态), 若无效则积分角速度
        q = msg.orientation
        self._imu_has_orientation = not (abs(q.x) < 1e-9 and abs(q.y) < 1e-9
                                         and abs(q.z) < 1e-9 and abs(q.w - 1.0) < 1e-9)
        if self._imu_has_orientation:
            self._imu_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        else:
            # Livox 等设备只给角速度, 自己积分
            now = time.monotonic()
            if self._last_imu_time is not None:
                dt = now - self._last_imu_time
                # 忽略异常大的 dt (>0.5s 认为是重启/丢帧, 不积分)
                if 0.0 < dt < 0.5:
                    self._imu_yaw += msg.angular_velocity.z * dt
            self._last_imu_time = now

    # ===================================================================
    # 控制循环
    # ===================================================================

    def _loop(self):
        """主控制循环：喂数据 → 评估状态机 → 执行动作"""
        if self.sm is None:
            return

        now = time.monotonic()

        # Livox 无 orientation: 积分上周期发布的 cmd_wz (桌面测试 IMU 不转时用指令驱动 yaw)
        if not self._imu_has_orientation and self._last_loop_time is not None:
            dt = now - self._last_loop_time
            if 0.0 < dt < 0.5:
                self._imu_yaw += self._last_cmd_wz * dt

        self.sm.update_imu(self._imu_yaw)

        tag = self._tag if self._tag_new else None
        self._tag_new = False

        r = self.sm.evaluate(tag, now)

        # -- 日志：状态变化时打印 --
        if self.sm.state != self._last_state:
            info = self.sm.info()
            tgt = f'z={info["target_z"]:.2f}m'
            if abs(info['target_x']) > 0.01:
                tgt += f' x={info["target_x"]:+.2f}m'
            step_str = ''
            if info['step_total'] > 0:
                step_str = f' step={info["step_idx"]+1}/{info["step_total"]}({info["step_type"]})'
            self.get_logger().info(
                f'[{info["state"]}] mission={info["mission_id"]} '
                f'tag_id={info["target_tag_id"]} '
                f'target({tgt}) dist={info["target_distance"]:.2f}m '
                f'prepare={info["prepare_dist"]:.2f}m '
                f'[{info["mission_idx"]+1}/{info["total"]}]{step_str}'
            )
            self._last_state = self.sm.state

        action = r.get('action', '')
        if action and action != self._last_action:
            self.get_logger().info(f'  → {action}')
            self._last_action = action

        # -- 速度指令 (只在伺服关闭时发，避免和 servo 冲突) --
        enable_servo = bool(r.get('enable_servo', False))
        cmd_wz = float(r['cmd_wz'])
        if not enable_servo and self._pub_cmd and self._pub_cmd.is_activated:
            t = Twist()
            t.linear.x  = float(r['cmd_vx'])
            t.linear.y  = float(r['cmd_vy'])
            t.angular.z = cmd_wz
            self._pub_cmd.publish(t)
        self._last_cmd_wz = cmd_wz
        self._last_loop_time = now

        # -- 远程设参 (带缓存，避免重复) --
        tid = int(r['target_tag_id'])
        if tid != self._cache_tag_id:
            self._set_remote(self._cli_detector, [('target_tag_id', tid)])
            self._cache_tag_id = tid

        if enable_servo != self._cache_enable:
            self._set_remote(self._cli_servo, [('enable', enable_servo)])
            self._cache_enable = enable_servo

        tdist = float(r['target_distance'])
        if self._cache_dist is None or abs(tdist - self._cache_dist) > 0.001:
            self._set_remote(self._cli_servo, [('target_distance', tdist)])
            self._cache_dist = tdist

        tx = float(r['target_x'])
        if self._cache_x is None or abs(tx - self._cache_x) > 0.001:
            self._set_remote(self._cli_servo, [('target_x', tx)])
            self._cache_x = tx

        # -- 发送任务信号 (服务调用, 有应答确认) --
        sig = r.get('mission_signal')
        if sig is not None and not self._signal_sent:
            self._pending_signal = sig
            self._call_trigger_service()
        elif sig is None:
            self._signal_sent = False    # 重置, 等下一个信号

    # ===================================================================
    # 辅助
    # ===================================================================

    def _set_remote(self, client, params):
        """
        调用远程 set_parameters 服务。

        client: rclpy Client (SetParameters)
        params: [(name, value), ...]
        """
        if client is None:
            return
        if not client.wait_for_service(timeout_sec=0.05):
            return

        req = SetParameters.Request()
        for name, value in params:
            pm = _make_param_msg(name, value)
            if pm is not None:
                req.parameters.append(pm)
        if not req.parameters:
            return

        try:
            client.call_async(req)
        except Exception:
            pass

    def _call_trigger_service(self):
        """
        调用外部控制板的 /mission_trigger 服务。

        成功 → 打印对方应答消息，标记已发送
        失败 → 下个循环周期自动重试
        """
        if self._cli_trigger is None or self._pending_signal is None:
            return

        if not self._cli_trigger.wait_for_service(timeout_sec=0.05):
            return   # 服务还没起来, 下个周期重试

        req = TriggerMission.Request()
        req.mission_id = int(self._pending_signal[0])
        req.tag_id     = int(self._pending_signal[1])
        req.distance   = float(self._pending_signal[2])
        req.angle      = float(self._pending_signal[3]) if len(self._pending_signal) > 3 else 0.0

        future = self._cli_trigger.call_async(req)

        # 轮询等待响应 (最多 1 秒)
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > 1.0:
                self.get_logger().warn('服务调用超时, 下周期重试')
                return
            rclpy.spin_once(self, timeout_sec=0.02)

        resp = future.result()
        if resp is not None and resp.success:
            self._signal_sent = True
            self.get_logger().info(
                f'*** 任务触发成功 *** '
                f'mission_id={req.mission_id}, tag_id={req.tag_id}, '
                f'distance={req.distance:.3f}m, angle={req.angle:.2f}rad | '
                f'应答: {resp.message}'
            )
        else:
            msg = resp.message if resp else '无响应'
            self.get_logger().warn(f'任务触发被拒绝: {msg}, 下周期重试')

    def _stop_robot(self):
        """发零速度"""
        t = Twist()
        try:
            if self._pub_cmd and self._pub_cmd.is_activated:
                self._pub_cmd.publish(t)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = MissionManagerNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

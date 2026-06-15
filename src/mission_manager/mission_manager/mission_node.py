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
from apriltag_interfaces.msg import TagPose, MissionSignal
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

        # 发布者 & 订阅者
        self._pub_cmd = None
        self._pub_signal = None
        self._sub_tag = None
        self._sub_imu = None

        # 远程参数客户端
        self._cli_detector = None          # → apriltag_detector/set_parameters
        self._cli_servo    = None          # → visual_servo_controller/set_parameters

        # 缓存上次设置的值，避免重复设参
        self._cache_tag_id = None
        self._cache_enable = None
        self._cache_dist   = None

        # 日志去重
        self._last_state = None

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
        self._pub_cmd = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)
        self._pub_signal = self.create_lifecycle_publisher(
            MissionSignal, '/mission_signal', 10
        )

        # -- 订阅者 --
        self._sub_tag = self.create_subscription(
            TagPose, '/tag_pose', self._cb_tag, 10
        )
        self._sub_imu = self.create_subscription(
            Imu, '/imu/data', self._cb_imu, 10
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
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._stop_robot()
        return TransitionCallbackReturn.SUCCESS

    # ===================================================================
    # 参数加载
    # ===================================================================

    def _declare_params(self):
        self.declare_parameter('missions', [
            {'mission_id': 1, 'tag_id': 0, 'stop_distance': 0.8},
            {'mission_id': 2, 'tag_id': 1, 'stop_distance': 1.0},
            {'mission_id': 3, 'tag_id': 2, 'stop_distance': 0.6},
        ])
        self.declare_parameter('prepare_distance_offset', 0.3)
        self.declare_parameter('tag_timeout', 2.0)
        self.declare_parameter('search_yaw_rate', 0.3)
        self.declare_parameter('blind_vx', 0.1)
        self.declare_parameter('blind_approach_timeout', 10.0)
        self.declare_parameter('kp_yaw', 0.5)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('signal_duration', 1.0)
        self.declare_parameter('loop_rate', 50.0)

    def _load_missions(self):
        """从参数中解析任务列表，兼容 dict 和 list 两种 YAML 格式"""
        raw = self.get_parameter('missions').value
        result = []
        for m in raw:
            if isinstance(m, dict):
                result.append({
                    'mission_id':    int(m.get('mission_id', 0)),
                    'tag_id':        int(m.get('tag_id', 0)),
                    'stop_distance': float(m.get('stop_distance', 0.5)),
                })
            elif isinstance(m, (list, tuple)):
                result.append({
                    'mission_id':    int(m[0]) if len(m) > 0 else 0,
                    'tag_id':        int(m[1]) if len(m) > 1 else 0,
                    'stop_distance': float(m[2]) if len(m) > 2 else 0.5,
                })
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
        self._imu_yaw = _yaw_from_quat(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        )

    # ===================================================================
    # 控制循环
    # ===================================================================

    def _loop(self):
        """主控制循环：喂数据 → 评估状态机 → 执行动作"""
        if self.sm is None:
            return

        now = time.monotonic()
        self.sm.update_imu(self._imu_yaw)

        tag = self._tag if self._tag_new else None
        self._tag_new = False

        r = self.sm.evaluate(tag, now)

        # -- 日志：状态变化时打印 --
        if self.sm.state != self._last_state:
            info = self.sm.info()
            self.get_logger().info(
                f'[{info["state"]}] mission={info["mission_id"]} '
                f'tag_id={info["target_tag_id"]} '
                f'target={info["target_distance"]}m '
                f'prepare={info["prepare_dist"]}m '
                f'[{info["mission_idx"]+1}/{info["total"]}]'
            )
            self._last_state = self.sm.state

        action = r.get('action', '')
        if action:
            self.get_logger().info(f'  → {action}')

        # -- 速度指令 (只在伺服关闭时发，避免和 servo 冲突) --
        enable_servo = bool(r.get('enable_servo', False))
        if not enable_servo and self._pub_cmd and self._pub_cmd.is_activated:
            t = Twist()
            t.linear.x  = float(r['cmd_vx'])
            t.linear.y  = float(r['cmd_vy'])
            t.angular.z = float(r['cmd_wz'])
            self._pub_cmd.publish(t)

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

        # -- 发布任务信号 --
        sig = r.get('mission_signal')
        if sig and self._pub_signal and self._pub_signal.is_activated:
            msg = MissionSignal()
            msg.mission_id = int(sig[0])
            msg.tag_id     = int(sig[1])
            msg.distance   = float(sig[2])
            self._pub_signal.publish(msg)
            self.get_logger().info(
                f'*** 任务信号已发送 *** '
                f'mission_id={msg.mission_id}, tag_id={msg.tag_id}, '
                f'distance={msg.distance:.3f}m'
            )

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

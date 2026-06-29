#!/usr/bin/env python3
"""
视觉伺服控制节点。

订阅 /tag_pose，根据 Tag 的位姿误差计算速度指令，发布到 /cmd_vel。
支持两种机器人：串联腿 (serial) 和并联腿 (parallel)。
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_interfaces.msg import TagPose
from .control_laws import SerialControlLaw, ParallelControlLaw

try:
    from scipy.spatial.transform import Rotation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


def yaw_from_quaternion(x, y, z, w):
    """从四元数中提取 yaw 角 (rad)，范围 [-pi, pi]"""
    if HAS_SCIPY:
        r = Rotation.from_quat([x, y, z, w])
        _, _, yaw = r.as_euler('xyz', degrees=False)
        return yaw

    # 手动计算
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class VisualServoNode(Node):
    """视觉伺服节点：Tag 位姿 → 速度指令"""

    def __init__(self):
        super().__init__('visual_servo_controller')

        # -- 声明参数 --
        self._declare_parameters()

        # -- 读取参数，构建控制律 --
        robot_type = self.get_parameter('robot_type').value
        self.enable = self.get_parameter('enable').value
        self.target_dist = self.get_parameter('target_distance').value
        max_v = self.get_parameter('max_linear_vel').value
        max_w = self.get_parameter('max_angular_vel').value

        if robot_type == 'serial':
            kp_dist = self.get_parameter('serial.kp_dist').value
            kp_x    = self.get_parameter('serial.kp_x').value
            kp_yaw  = self.get_parameter('serial.kp_yaw').value
            self.law = SerialControlLaw(kp_dist, kp_x, kp_yaw, max_v, max_w)
            self.get_logger().info(
                f'使用 SerialControlLaw: '
                f'kp_dist={kp_dist}, kp_x={kp_x}, kp_yaw={kp_yaw}'
            )

        elif robot_type == 'parallel':
            kp_yaw  = self.get_parameter('parallel.kp_yaw').value
            kp_x    = self.get_parameter('parallel.kp_x').value
            kp_dist = self.get_parameter('parallel.kp_dist').value
            align_t = self.get_parameter('parallel.align_threshold').value
            yaw_t   = self.get_parameter('parallel.align_yaw_threshold').value
            self.law = ParallelControlLaw(
                kp_yaw, kp_x, kp_dist, align_t, yaw_t, max_v, max_w
            )
            self.get_logger().info(
                f'使用 ParallelControlLaw: '
                f'kp_yaw={kp_yaw}, kp_x={kp_x}, kp_dist={kp_dist}'
            )

        else:
            raise ValueError(
                f'不支持的 robot_type: "{robot_type}"，'
                f'可选: "serial" 或 "parallel"'
            )

        self.robot_type = robot_type

        # -- 发布 & 订阅 --
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.tag_sub = self.create_subscription(
            TagPose, '/tag_pose', self._on_tag_pose, 10
        )

        # -- 超时保护：太久没看到 Tag 就停车 --
        self._last_seen = self.get_clock().now()
        self._timeout = 0.5                        # 0.5 秒
        self._timer = self.create_timer(0.1, self._check_timeout)

        # -- 节流日志 --
        self._last_log_time = self.get_clock().now()
        self._log_interval = 2.0

        # -- 参数更新回调 --
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'视觉伺服节点就绪 (type={robot_type}, '
            f'target={self.target_dist}m, enabled={self.enable})'
        )

    # -------------------------------------------------------------------
    # 参数
    # -------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('robot_type', 'serial')
        self.declare_parameter('serial.kp_dist', 0.5)
        self.declare_parameter('serial.kp_x', 0.3)
        self.declare_parameter('serial.kp_yaw', 1.0)
        self.declare_parameter('parallel.kp_yaw', 1.0)
        self.declare_parameter('parallel.kp_x', 0.4)
        self.declare_parameter('parallel.kp_dist', 0.3)
        self.declare_parameter('parallel.align_threshold', 0.05)
        self.declare_parameter('parallel.align_yaw_threshold', 0.1)
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('enable', True)

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'enable':
                self.enable = p.value
                self.get_logger().info(f'enable → {self.enable}')
                if not self.enable:
                    self._stop()
            elif p.name == 'target_distance':
                self.target_dist = p.value
                self.get_logger().info(f'target_distance → {self.target_dist}')
                if self.robot_type == 'parallel':
                    self.law.reset()
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------------
    # 主逻辑
    # -------------------------------------------------------------------

    def _on_tag_pose(self, msg):
        """收到 Tag 位姿 → 计算并发布速度指令"""
        self._last_seen = self.get_clock().now()

        if not self.enable:
            return

        # 提取 x, z, yaw
        tag_x = msg.pose.position.x
        tag_z = msg.pose.position.z
        tag_yaw = yaw_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        twist = Twist()

        if self.robot_type == 'serial':
            vx, vy, wz = self.law.compute(tag_x, tag_z, tag_yaw, self.target_dist)
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
        else:
            vx, wz, changed, state_name, e_x, e_yaw, e_z = self.law.compute(
                tag_x, tag_z, tag_yaw, self.target_dist
            )
            twist.linear.x = vx
            twist.linear.y = 0.0
            twist.angular.z = wz

            # 状态切换时立即打印
            if changed:
                self.get_logger().info(
                    f'伺服状态: {state_name} | '
                    f'e_x={e_x:.3f}m e_yaw={e_yaw:.2f}rad e_z={e_z:.2f}m'
                )

            # 节流日志
            now = self.get_clock().now()
            dt = (now - self._last_log_time).nanoseconds / 1e9
            if dt >= self._log_interval:
                self.get_logger().info(
                    f'伺服状态: {state_name} | '
                    f'Vx={vx:.3f} Wz={wz:.3f} | '
                    f'e_x={e_x:.3f}m e_yaw={e_yaw:.2f}rad e_z={e_z:.2f}m'
                )
                self._last_log_time = now

        self.cmd_pub.publish(twist)

    def _check_timeout(self):
        """超时保护：太久没收到 Tag 就发零速度停车"""
        if not self.enable:
            return
        dt = (self.get_clock().now() - self._last_seen).nanoseconds * 1e-9
        if dt > self._timeout:
            self._stop()

    def _stop(self):
        """发送零速度"""
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

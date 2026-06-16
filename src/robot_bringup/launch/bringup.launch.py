"""
bringup.launch.py

启动 Apriltag 任务触发系统的全部节点:
  1. apriltag_detector       — Tag 检测
  2. visual_servo_controller — 视觉伺服
  3. mission_manager         — 任务状态机 (LifecycleNode)

mission_manager 是 LifecycleNode，启动后需要手动 configure + activate。
这里用一个 OpaqueFunction 在节点就绪后自动完成生命周期切换。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy


def _activate_mission_manager(context, *args, **kwargs):
    """
    等待 mission_manager 就绪，然后依次调用:
      configure → activate
    """
    rclpy.init(args=None)
    node = rclpy.create_node('bringup_lifecycle_helper')
    cli = node.create_client(ChangeState, '/mission_manager/change_state')

    node.get_logger().info('等待 mission_manager 的 ChangeState 服务...')
    if not cli.wait_for_service(timeout_sec=15.0):
        node.get_logger().error('ChangeState 服务超时 (15s)！')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Step 1: configure
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_CONFIGURE
    node.get_logger().info('→ configure')
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if fut.result() is None or not fut.result().success:
        node.get_logger().error('configure 失败！')
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info('configure 成功')

    # Step 2: activate
    req2 = ChangeState.Request()
    req2.transition.id = Transition.TRANSITION_ACTIVATE
    node.get_logger().info('→ activate')
    fut2 = cli.call_async(req2)
    rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
    if fut2.result() is None or not fut2.result().success:
        node.get_logger().error('activate 失败！')
    else:
        node.get_logger().info('activate 成功 — 任务管理器已运行')

    node.destroy_node()
    rclpy.shutdown()


def generate_launch_description():
    """生成完整的 launch 描述"""

    # -- 配置文件路径 --
    pkg = FindPackageShare('robot_bringup')
    cfg = lambda f: PathJoinSubstitution([pkg, 'config', f])

    # -- 启动参数 --
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='serial',
        description='机器人类型: "serial" (串联腿) 或 "parallel" (并联腿)'
    )

    # -- 节点 1: Tag 检测 --
    detector = Node(
        package='apriltag_detector',
        executable='detector_node',
        name='apriltag_detector',
        output='screen',
        parameters=[cfg('detector_params.yaml'), cfg('camera_tf.yaml')],
    )

    # -- 节点 2: 视觉伺服 --
    servo = Node(
        package='visual_servo_controller',
        executable='servo_node',
        name='visual_servo_controller',
        output='screen',
        parameters=[
            cfg('controller_params.yaml'),
            {'robot_type': LaunchConfiguration('robot_type')},
        ],
    )

    # -- 节点 3: 任务管理器 (LifecycleNode) --
    # missions.yaml 不用 ROS2 --params-file 机制加载（嵌套结构不兼容），
    # 改为通过 missions_file 参数传路径，节点内部用 yaml.safe_load 读取
    mission = LifecycleNode(
        package='mission_manager',
        executable='mission_node',
        name='mission_manager',
        namespace='',
        output='screen',
        parameters=[
            cfg('mission_params.yaml'),
            {'missions_file': cfg('missions.yaml')},
        ],
    )

    # -- 生命周期助手: 延迟 3 秒后自动 configure + activate --
    activator = TimerAction(
        period=3.0,
        actions=[OpaqueFunction(function=_activate_mission_manager)],
    )

    return LaunchDescription([
        robot_type_arg,
        detector,
        servo,
        mission,
        activator,
    ])

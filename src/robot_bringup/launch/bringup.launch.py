"""
bringup.launch.py

启动 Apriltag 任务触发系统的全部节点:
  1. apriltag_detector       — Tag 检测
  2. visual_servo_controller — 视觉伺服
  3. mission_manager         — 任务状态机 (LifecycleNode)

mission_manager 是 LifecycleNode，启动后自动完成 configure + activate。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.actions import LifecycleTransition
from launch_ros.substitutions import FindPackageShare


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

    # -- 自动生命周期切换: configure → activate --
    configure_evt = LifecycleTransition(
        lifecycle_node_names=['mission_manager'],
        transition_ids=[
            'TRANSITION_CONFIGURE',
            'TRANSITION_ACTIVATE',
        ],
    )

    # 延迟 3 秒等节点就绪后执行
    activator = TimerAction(period=3.0, actions=[configure_evt])

    return LaunchDescription([
        robot_type_arg,
        detector,
        servo,
        mission,
        activator,
    ])

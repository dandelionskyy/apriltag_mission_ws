#!/usr/bin/env python3
"""
AprilTag 检测节点。

从 D455 相机读取图像，用 pupil_apriltags 库检测 AprilTag，
发布目标 Tag 的 3D 位姿和全部检测结果。
同时广播 base_link -> camera_link 的静态 TF。
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
from apriltag_interfaces.msg import TagPose, TagDetection, TagDetectionArray

# pupil_apriltags 和 scipy 是可选的，没装就给个清晰的报错
try:
    from pupil_apriltags import Detector
    HAS_PUPIL = True
except ImportError:
    HAS_PUPIL = False

try:
    from scipy.spatial.transform import Rotation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ---------------------------------------------------------------------------
# 工具函数：矩阵 / 四元数 / 欧拉角 转换
# ---------------------------------------------------------------------------

def matrix_to_quaternion(R):
    """
    3x3 旋转矩阵 → 四元数 (x, y, z, w)

    优先用 scipy，没有的话用手动实现兜底。
    """
    if HAS_SCIPY:
        r = Rotation.from_matrix(R)
        q = r.as_quat()          # scipy 返回 [x, y, z, w]
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])

    # 手动转换，参考 euclideanspace.com
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return float(x), float(y), float(z), float(w)


def rpy_to_quaternion(roll, pitch, yaw):
    """欧拉角 (rad) → 四元数 (x, y, z, w)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return float(x), float(y), float(z), float(w)


# ---------------------------------------------------------------------------
# 检测节点
# ---------------------------------------------------------------------------

class AprilTagDetectorNode(Node):
    """AprilTag 检测节点：订阅图像，检测 Tag，发布位姿和 TF"""

    def __init__(self):
        super().__init__('apriltag_detector')

        if not HAS_PUPIL:
            self.get_logger().fatal(
                'pupil_apriltags 未安装，请执行: '
                'pip install pupil-apriltags'
            )
            raise ImportError('pupil_apriltags 不可用')

        # -- 声明参数 --
        self._declare_parameters()

        # -- 初始化 pupil_apriltags 检测器 --
        families = self.get_parameter('families').value
        nthreads = self.get_parameter('nthreads').value
        self.tag_size = self.get_parameter('tag_size').value

        self.get_logger().info(
            f'初始化检测器: families={families}, '
            f'tag_size={self.tag_size}m, nthreads={nthreads}'
        )

        self.detector = Detector(
            families=families,
            nthreads=nthreads,
            quad_decimate=self.get_parameter('quad_decimate').value,
            quad_sigma=self.get_parameter('quad_sigma').value,
            refine_edges=self.get_parameter('refine_edges').value,
            decode_sharpening=self.get_parameter('decode_sharpening').value,
            debug=self.get_parameter('debug').value,
        )

        # 相机内参
        self.fx = self.get_parameter('camera_intrinsics.fx').value
        self.fy = self.get_parameter('camera_intrinsics.fy').value
        self.cx = self.get_parameter('camera_intrinsics.cx').value
        self.cy = self.get_parameter('camera_intrinsics.cy').value
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]

        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.use_camera_info = self.get_parameter('use_camera_info_topic').value

        # -- CV Bridge --
        self.bridge = CvBridge()

        # -- 发布者 --
        self.tag_pose_pub = self.create_publisher(TagPose, '/tag_pose', 10)
        self.tag_detections_pub = self.create_publisher(
            TagDetectionArray, '/tag_detections', 10
        )

        # -- 订阅者 --
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )
        self.get_logger().info(f'订阅图像话题: {image_topic}')

        if self.use_camera_info:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, '/camera/color/camera_info',
                self._on_camera_info, 10
            )
            self.get_logger().info('订阅 CameraInfo，内参将从话题自动获取')

        # -- 静态 TF: base_link → camera_link --
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_camera_tf()

        # -- 动态参数回调 --
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info('AprilTag 检测节点初始化完成')

    # -------------------------------------------------------------------
    # 参数
    # -------------------------------------------------------------------

    def _declare_parameters(self):
        """声明所有可配置参数"""
        # 检测器参数
        self.declare_parameter('families', 'tag36h11')
        self.declare_parameter('nthreads', 4)
        self.declare_parameter('quad_decimate', 1.0)
        self.declare_parameter('quad_sigma', 0.0)
        self.declare_parameter('refine_edges', 1)
        self.declare_parameter('decode_sharpening', 0.25)
        self.declare_parameter('debug', 0)

        # Tag 物理尺寸
        self.declare_parameter('tag_size', 0.16)

        # 相机内参
        self.declare_parameter('camera_intrinsics.fx', 615.0)
        self.declare_parameter('camera_intrinsics.fy', 615.0)
        self.declare_parameter('camera_intrinsics.cx', 320.0)
        self.declare_parameter('camera_intrinsics.cy', 240.0)
        self.declare_parameter('use_camera_info_topic', False)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('target_tag_id', -1)

        # 相机外参 (TF)
        self.declare_parameter('camera_tf.translation.x', 0.18)
        self.declare_parameter('camera_tf.translation.y', 0.0)
        self.declare_parameter('camera_tf.translation.z', 0.25)
        self.declare_parameter('camera_tf.rotation.roll', 0.0)
        self.declare_parameter('camera_tf.rotation.pitch', 0.0)
        self.declare_parameter('camera_tf.rotation.yaw', 0.0)

    def _on_param_change(self, params):
        """动态参数更新回调 —— 目前只关心 target_tag_id 的变化"""
        for p in params:
            if p.name == 'target_tag_id':
                self.target_tag_id = p.value
                self.get_logger().info(
                    f'target_tag_id 已更新为 {self.target_tag_id}'
                )
        return True

    # -------------------------------------------------------------------
    # TF 广播
    # -------------------------------------------------------------------

    def _broadcast_camera_tf(self):
        """读取 YAML 配置并广播 base_link → camera_link 的静态变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = float(
            self.get_parameter('camera_tf.translation.x').value)
        t.transform.translation.y = float(
            self.get_parameter('camera_tf.translation.y').value)
        t.transform.translation.z = float(
            self.get_parameter('camera_tf.translation.z').value)

        roll  = self.get_parameter('camera_tf.rotation.roll').value
        pitch = self.get_parameter('camera_tf.rotation.pitch').value
        yaw   = self.get_parameter('camera_tf.rotation.yaw').value

        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'已发布静态 TF: base_link → camera_link '
            f'(x={t.transform.translation.x:.3f}, '
            f'y={t.transform.translation.y:.3f}, '
            f'z={t.transform.translation.z:.3f})'
        )

    # -------------------------------------------------------------------
    # 订阅回调
    # -------------------------------------------------------------------

    def _on_camera_info(self, msg):
        """从 CameraInfo 话题更新相机内参"""
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]

        self.get_logger().info(
            f'从 CameraInfo 更新内参: '
            f'fx={self.fx:.1f}, fy={self.fy:.1f}, '
            f'cx={self.cx:.1f}, cy={self.cy:.1f}'
        )
        # 只打印一次，之后不再订阅
        self.use_camera_info = False

    def _on_image(self, msg):
        """
        图像回调：转灰度图 → 检测 Tag → 发布结果

        对每个检测到的 Tag，用 pupil_apriltags 返回的 pose_R / pose_t
        构建 geometry_msgs/Pose，发布到 /tag_pose (仅目标 Tag)
        和 /tag_detections (全部 Tag)。
        """
        # 1. 转灰度图
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 转换失败: {e}')
            return

        # 2. 检测 Tag
        try:
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size,
            )
        except Exception as e:
            self.get_logger().error(f'Tag 检测异常: {e}')
            return

        # 3. 没有检测到任何 Tag
        if not tags:
            empty = TagDetectionArray(header=msg.header, detections=[])
            self.tag_detections_pub.publish(empty)
            return

        # 4. 遍历检测结果
        detections = []
        best_pose = None          # 目标 Tag 的位姿 (选最近的)
        best_z = float('inf')

        for tag in tags:
            # 构建单个 Tag 的位姿消息
            pose_msg = TagPose()
            pose_msg.tag_id = tag.tag_id

            # pose_t: 相机坐标系下的平移 [x_right, y_down, z_forward]
            pose_msg.pose.position.x = float(tag.pose_t[0])
            pose_msg.pose.position.y = float(tag.pose_t[1])
            pose_msg.pose.position.z = float(tag.pose_t[2])

            qx, qy, qz, qw = matrix_to_quaternion(tag.pose_R)
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            # 构建检测记录 (用于 /tag_detections)
            det = TagDetection()
            det.tag_id = tag.tag_id
            if tag.corners is not None and len(tag.corners) == 4:
                det.corners_x = [float(c[0]) for c in tag.corners]
                det.corners_y = [float(c[1]) for c in tag.corners]
            det.decision_margin = float(tag.decision_margin)
            det.pose = pose_msg.pose
            detections.append(det)

            # 筛选目标 Tag (匹配 target_tag_id，取最近的那个)
            if self.target_tag_id < 0 or tag.tag_id == self.target_tag_id:
                z = float(tag.pose_t[2])
                if z < best_z:
                    best_z = z
                    best_pose = pose_msg

        # 5. 发布全部检测结果
        self.tag_detections_pub.publish(
            TagDetectionArray(header=msg.header, detections=detections)
        )

        # 6. 发布目标 Tag 的位姿
        if best_pose is not None:
            self.tag_pose_pub.publish(best_pose)
        else:
            self.get_logger().debug(
                f'检测到 {len(tags)} 个 Tag，但目标 ID '
                f'(target_tag_id={self.target_tag_id}) 不在其中'
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AprilTagDetectorNode()
        rclpy.spin(node)
    except ImportError as e:
        rclpy.logging.get_logger('apriltag_detector').fatal(str(e))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

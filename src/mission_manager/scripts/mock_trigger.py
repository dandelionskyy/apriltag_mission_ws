#!/usr/bin/env python3
"""
Mock /mission_trigger 服务端 — 模拟外部控制板应答。

用法:
    python3 scripts/mock_trigger.py
    # 或者
    ros2 run mission_manager mock_trigger.py
"""

import rclpy
from rclpy.node import Node
from apriltag_interfaces.srv import TriggerMission

# mission_id → 障碍类型  (与 state_machine.py 中 ObstacleType 一致)
OBSTACLE_MAP = {
    0:  "DEFAULT_WALK",
    1:  "CRAWLING_FRAME",
    2:  "HIGH_WALL",
    3:  "STAIR",
    4:  "SANDPIT",
    5:  "SLOPE",
}


class MockTrigger(Node):
    def __init__(self):
        super().__init__('mock_trigger')
        self.srv = self.create_service(
            TriggerMission, '/mission_signal', self._on_trigger
        )
        self.get_logger().info('MockTrigger 已就绪, 等待 /mission_trigger 请求...')

    def _on_trigger(self, request, response):
        obstacle = OBSTACLE_MAP.get(request.mission_id, f"UNKNOWN({request.mission_id})")
        self.get_logger().info(
            f'>>> 收到触发请求 <<<\n'
            f'    mission_id = {request.mission_id}  ({obstacle})\n'
            f'    tag_id     = {request.tag_id}\n'
            f'    distance   = {request.distance:.3f} m\n'
            f'    angle      = {request.angle:.3f} rad ({request.angle*57.3:.1f}°)'
        )
        response.success = True
        response.message = f'OK — mock 应答, obstacle={obstacle}'
        return response


def main():
    rclpy.init()
    node = MockTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

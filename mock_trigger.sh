#!/bin/bash
# 启动 mock_trigger 服务端 (/mission_signal)
# 模拟外部控制板, 收到请求后打印并返回 success

cd ~/apriltag_mission_ws
source install/setup.bash
python3 src/mission_manager/scripts/mock_trigger.py

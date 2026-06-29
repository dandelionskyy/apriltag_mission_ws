#!/bin/bash
# 一键启动 apriltag 任务系统 (detector + servo + mission_manager)

cd ~/Dog_ROS2
source install/setup.bash
cd ~/apriltag_mission_ws
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py robot_type:=parallel

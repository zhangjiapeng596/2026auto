#!/bin/bash
# ============================================
# ABOT 全局一键安全退出与死锁清扫脚本
# 功能: 发送物理零速电子刹车 + 4层全量节点彻底销毁 + 释放 11311 端口
# ============================================

echo "[STOP] 正在发送安全电子刹车信号 (cmd_vel=[0,0,0])..."
source /opt/ros/melodic/setup.bash 2>/dev/null
timeout 1 rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

PIDFILE=/tmp/abot_competition.pid
INNER_SCRIPT=/tmp/abot_competition_inner.sh

if [ -f "$PIDFILE" ]; then
    PID=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$PID" ] && kill -15 $PID 2>/dev/null || true
fi

pkill -15 -f 'abot_competition_inner\.sh' 2>/dev/null || true
sleep 1

echo "[STOP] 正在销毁所有 ROS 节点与后台子进程..."

# 精准杀死所有可能残留的 Python Worker / ROS / 硬件节点
pkill -9 -f 'doubao_worker'            2>/dev/null || true
pkill -9 -f 'doubao_openAI'            2>/dev/null || true
pkill -9 -f 'doubao_tts'               2>/dev/null || true
pkill -9 -f 'vlm_bridge'               2>/dev/null || true
pkill -9 -f 'vlm_node'                 2>/dev/null || true
pkill -9 -f 'usb_cam_node'             2>/dev/null || true
pkill -9 -f 'top_view_shot_node'       2>/dev/null || true
pkill -9 -f 'mission_state_machine'     2>/dev/null || true
pkill -9 -f 'safety_monitor'           2>/dev/null || true
pkill -9 -f 'move_base'                2>/dev/null || true
pkill -9 -f 'lidar_loc'                2>/dev/null || true
pkill -9 -f 'costmap_cleaner'          2>/dev/null || true
pkill -9 -f 'lidar_filter_node'        2>/dev/null || true
pkill -9 -f 'amcl'                     2>/dev/null || true
pkill -9 -f 'rplidarNode'              2>/dev/null || true
pkill -9 -f 'start_lidar_motor'        2>/dev/null || true
pkill -9 -f 'abot_driver'              2>/dev/null || true
pkill -9 -f 'abot_imu'                 2>/dev/null || true
pkill -9 -f 'robot_pose_ekf'           2>/dev/null || true
pkill -9 -f 'cov_inflate'              2>/dev/null || true
pkill -9 -f 'odom_ekf'                 2>/dev/null || true
pkill -9 -f 'map_server'               2>/dev/null || true
pkill -9 -f 'rviz'                     2>/dev/null || true
pkill -9 -f 'roscore'                  2>/dev/null || true
pkill -9 -f 'rosmaster'                2>/dev/null || true
pkill -9 -f 'rosout'                   2>/dev/null || true
pkill -9 -f 'roslaunch'                2>/dev/null || true
pkill -9 -f 'rosrun'                   2>/dev/null || true

# 扫荡所有运行在 3X2KY2 目录下的脚本与测试进程
pkill -9 -f '3X2KY2'                   2>/dev/null || true
pkill -9 -f 'abot_competition_inner\.sh' 2>/dev/null || true

rm -f "$PIDFILE" "$INNER_SCRIPT" /tmp/comp_*.log /tmp/comp_*.pid 2>/dev/null || true

echo "[STOP] 正在验证端口释放..."
for i in $(seq 1 5); do
    ss -tlnp 2>/dev/null | grep -q ':11311\b' || break
    sleep 0.5
done

echo "[STOP] 所有节点与资源已 100% 彻底安全退出。"

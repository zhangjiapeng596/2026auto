#!/bin/bash
# ============================================
# ABOT 全局一键安全退出与死锁清扫脚本
# (适用于 按 Enter 退出、Ctrl+C 退出、--stop 远端退出)
# ============================================

ts=$(date '+%H:%M:%S' 2>/dev/null || echo "??:??:??")
echo "[$ts] [STOP] 收到退出指令，发送安全电子刹车 (cmd_vel=[0,0,0])..."
source /opt/ros/melodic/setup.bash 2>/dev/null
timeout 1 rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

PIDFILE=/tmp/abot_competition.pid
INNER_SCRIPT=/tmp/abot_competition_inner.sh

if [ -f "$PIDFILE" ]; then
    PID=$(cat "$PIDFILE" 2>/dev/null)
    [ -n "$PID" ] && kill -15 $PID 2>/dev/null || true
fi

pkill -15 -f 'abot_competition_inner\.sh' 2>/dev/null || true
sleep 0.5

echo "[$ts] [STOP] 正在彻底销毁所有 ROS 节点与后台进程..."

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

# 扫荡 3X2KY2 目录下所有关联运行进程
pkill -9 -f '3X2KY2'                   2>/dev/null || true

# 清楚 PID 与临时脚本文件
rm -f "$PIDFILE" "$INNER_SCRIPT" /tmp/comp_*.log /tmp/comp_*.pid 2>/dev/null || true

echo "[$ts] [STOP] 验证 ROS Master 端口 (11311)..."
for i in $(seq 1 5); do
    ss -tlnp 2>/dev/null | grep -q ':11311\' || break
    sleep 0.3
done

echo "[$ts] [STOP] 所有节点与窗口已 100% 彻底销毁退出。"

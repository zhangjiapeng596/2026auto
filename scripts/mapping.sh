#!/bin/bash
# ============================================
# 模式一：键盘控制建图
# 用途：手动操控机器人在场地中移动，SLAM 实时构建地图
#
# 节点列表 (8个):
#   roscore + abot_driver + abot_imu + rplidar
#   + box_filter + robot_state_publisher
#   + slam_gmapping + teleop_keyboard
#
# 进程管理:
#   trap EXIT INT TERM 防僵尸进程 — Ctrl+C 或异常退出自动清理
# ============================================

WS_PATH="${WS_PATH:-${HOME}/3X2KY2}"
MAP_NAME="${1:-game}"

# ============================================
# PID 追踪与清理
# ============================================
TRACKED_PIDS=""

track_pid() {
    TRACKED_PIDS="$TRACKED_PIDS $1"
}

cleanup_all() {
    echo ""
    echo "[清理] 停止建图节点..."
    # L1: SIGTERM
    for p in $TRACKED_PIDS; do
        kill $p 2>/dev/null
    done
    sleep 3
    # L2: SIGKILL
    for p in $TRACKED_PIDS; do
        kill -9 $p 2>/dev/null
    done
    # L3: 兜底
    pkill -9 -f 'roscore'           2>/dev/null || true
    pkill -9 -f 'rosmaster'         2>/dev/null || true
    pkill -9 -f 'rosout'            2>/dev/null || true
    pkill -9 -f 'roslaunch'         2>/dev/null || true
    pkill -9 -f 'rosrun'            2>/dev/null || true
    pkill -9 -f 'rplidarNode'       2>/dev/null || true
    pkill -9 -f 'slam_gmapping'     2>/dev/null || true
    pkill -9 -f 'abot_driver'       2>/dev/null || true
    pkill -9 -f 'abot_imu'          2>/dev/null || true
    pkill -9 -f 'teleop_twist'      2>/dev/null || true
    pkill -9 -f 'box_filter'        2>/dev/null || true
    pkill -9 -f 'robot_state_pub'   2>/dev/null || true
    echo "[清理] 完成"
}

trap cleanup_all EXIT INT TERM

echo "=== ABOT 键盘控制建图 ==="
echo "地图: ${WS_PATH}/src/robot_slam/maps/${MAP_NAME}"
echo "Ctrl+C 停止并清理所有节点"
echo "完成后运行: rosrun map_server map_saver -f ${WS_PATH}/src/robot_slam/maps/${MAP_NAME}"

# 设置 DISPLAY（实车 GNOME 桌面需要）
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# 检查是否为 WSL 环境
if grep -qi microsoft /proc/version 2>/dev/null; then
    # ===== WSL 模式：后台启动，无图形界面 =====
    echo "[WSL] 后台启动模式..."

    roscore &
    track_pid $!
    sleep 2

    source /opt/ros/melodic/setup.bash
    source ${WS_PATH}/devel/setup.bash

    roslaunch abot_bringup robot.launch &
    track_pid $!
    sleep 5

    roslaunch robot_slam gmapping.launch &
    track_pid $!
    sleep 5

    echo "=== 全部节点已启动 ==="
    echo "在另一个终端运行键盘控制:"
    echo "  source ${WS_PATH}/devel/setup.bash"
    echo "  rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
    echo "保存地图:"
    echo "  rosrun map_server map_saver -f ${WS_PATH}/src/robot_slam/maps/${MAP_NAME}"
    wait

else
    # ===== 实车模式：GNOME 终端分窗口启动 =====
    gnome-terminal -- bash -c '
        source /opt/ros/melodic/setup.bash
        roscore
        exec bash' &
    track_pid $!
    sleep 2

    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        sleep 3
        roslaunch abot_bringup robot.launch
        exec bash" &
    track_pid $!
    sleep 2

    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        sleep 8
        roslaunch robot_slam gmapping.launch
        exec bash" &
    track_pid $!
    sleep 2

    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        sleep 10
        roslaunch robot_slam view_mapping.launch
        exec bash" &
    track_pid $!
    sleep 2

    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        sleep 12
        rosrun teleop_twist_keyboard teleop_twist_keyboard.py
        exec bash" &
    track_pid $!
    sleep 2

    echo "=== 5 个终端窗口已启动 ==="
    echo "1. roscore"
    echo "2. 底盘驱动 (abot_driver + IMU + LiDAR)"
    echo "3. Gmapping SLAM"
    echo "4. RViz 可视化"
    echo "5. 键盘控制"
    echo ""
    echo "保存地图: rosrun map_server map_saver -f ${WS_PATH}/src/robot_slam/maps/${MAP_NAME}"
    echo "关闭所有 GNOME 终端窗口或 Ctrl+C 停止全部"
    wait
fi

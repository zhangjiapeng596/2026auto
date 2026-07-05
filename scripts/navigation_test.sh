#!/bin/bash
# ============================================
# 模式 2：纯导航测试
# 用途: 加载先验地图, 按预设路径点顺序导航, 验证导航精度
#   用法: bash navigation_test.sh [地图名] [路径脚本]
#   节点: roscore + bringup(IMU) + nav(AMCL+map+move_base) + multi_goals + RViz
#   无视觉/语音/唤醒词
#
# 进程管理:
#   trap EXIT INT TERM 防僵尸进程 — Ctrl+C 或异常退出自动清理
# ============================================

WS_PATH="${WS_PATH:-${HOME}/3X2KY2_backup2}"
# 地图名不带 .yaml 则自动补全
MAP_NAME="${1:-competition_field}"
[[ "$MAP_NAME" != *.yaml ]] && MAP_NAME="${MAP_NAME}.yaml"
GOALS_SCRIPT="${2:-nav_vision_goals.py}"

# ============================================
# PID 追踪与清理
# ============================================
TRACKED_PIDS=""

track_pid() {
    TRACKED_PIDS="$TRACKED_PIDS $1"
}

cleanup_all() {
    echo ""
    echo "[清理] 停止导航测试节点..."
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
    pkill -9 -f 'move_base'         2>/dev/null || true
    pkill -9 -f 'amcl'              2>/dev/null || true
    pkill -9 -f 'map_server'        2>/dev/null || true
    pkill -9 -f 'abot_driver'       2>/dev/null || true
    pkill -9 -f 'abot_imu'          2>/dev/null || true
    pkill -9 -f 'robot_pose_ekf'    2>/dev/null || true
    pkill -9 -f 'rviz'              2>/dev/null || true
    pkill -9 -f 'nav_vision_goals'  2>/dev/null || true
    pkill -9 -f 'navigation_multi'  2>/dev/null || true
    echo "[清理] 完成"
}

trap cleanup_all EXIT INT TERM

echo "=== ABOT 预设路径导航测试 ==="
echo "地图: ${MAP_NAME}"
echo "路径脚本: ${GOALS_SCRIPT}"
echo "Ctrl+C 停止并清理所有节点"

# ============================================
# 工作空间编译检查（铁律：只 source /opt/ros/melodic，禁止 source 原厂 ~/abot_ws/）
# ============================================
if [ ! -f "${WS_PATH}/devel/setup.bash" ]; then
    echo ""
    echo "!!! 错误: ${WS_PATH}/devel/setup.bash 不存在！"
    echo "!!! 请先编译开发工作空间（注意：禁止 source 原厂 ~/abot_ws/）："
    echo ""
    echo "    source /opt/ros/melodic/setup.bash"
    echo "    cd ${WS_PATH} && catkin_make"
    echo ""
    exit 1
fi

# 检查关键包是否可找到
source /opt/ros/melodic/setup.bash
source ${WS_PATH}/devel/setup.bash
for pkg in abot_model robot_slam abot_bringup lidar_filters; do
    if ! rospack find $pkg > /dev/null 2>&1; then
        echo "!!! 错误: 找不到 ROS 包 '$pkg'，请检查编译是否成功"
        exit 1
    fi
done
echo "  工作空间检查通过"

# 重新设置 PATH（source 后可能被覆盖）
export PATH="/opt/ros/melodic/bin:$(echo "$PATH" | sed -e 's|/home/abot/anaconda3[^:]*:||g' -e 's|:/home/abot/anaconda3[^:]*||g')"

export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# ============================================
# ROS 网络环境锁定（修复 SSH 多会话 master 不可达）
# 问题: 不同 SSH 会话默认 ROS_MASTER_URI 不一致，roslaunch 找不到
#       master 就自启 roscore，导致多个 rosmaster 共存、节点互相不可见。
# 修复: 显式绑定 localhost:11311，roslaunch 不再自启 roscore。
# 监控侧同步使用 localhost 确保话题/服务可达。
# ============================================
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 清理旧进程
echo "=== 清理旧进程 ==="
killall -9 roslaunch roscore rosmaster rosout 2>/dev/null || true
sleep 2

echo "=== 启动 roscore ==="
roscore &
track_pid $!
sleep 4

echo "=== 启动底盘驱动 ==="
roslaunch abot_bringup robot_with_imu.launch &
track_pid $!
sleep 10

echo "=== 启动导航栈 ==="
roslaunch robot_slam navigation.launch map_name:=${MAP_NAME} &
track_pid $!
sleep 12

echo "=== 启动 RViz ==="
rosrun rviz rviz -d ${WS_PATH}/src/robot_slam/rviz/view_navigation.rviz &
track_pid $!
sleep 3

echo ""
echo "=== 全部启动完成 ==="
echo "节点:"
rosnode list 2>&1
echo ""
echo "关键话题:"
rostopic list 2>&1 | grep -E "/map$|/odom$|/scan_filtered$|/amcl_pose$"

echo ""
echo "=== 发送初始位姿 (Cell 1) ==="
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position: {x: -1.6, y: 1.6, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
" -1
sleep 2

echo ""
echo "=== 启动预设路径导航 ==="
rosrun robot_slam ${GOALS_SCRIPT}
# rosrun 前台运行，Ctrl+C 或脚本结束 → trap 清理
wait

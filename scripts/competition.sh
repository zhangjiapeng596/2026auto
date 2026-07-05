#!/bin/bash
# ============================================
# 模式 1 / 模式 3：完整比赛 / 导航+视觉+语音
#
# 模式 1 (sim_mode=false): 完整比赛 — Snowboy 唤醒 + 导航 + 豆包VLM + TTS播报
#   用法: bash scripts/competition.sh game false
#   节点: roscore + bringup(IMU) + nav(lidar_loc+map+move_base) + GameStart(Snowboy)
#         + VLM(doubao) + TTS(doubao) + state_machine(sim_mode=false) + safety
#
# 模式 3 (sim_mode=true):  导航+视觉+语音 — 无唤醒词, 5s 自动开始
#   用法: bash scripts/competition.sh game true
#   节点: 同上但不启动 GameStart, 状态机 5s 后自动进入流程
#
# SSH 启动: 自动检测 SSH_TTY，使用 setsid 后台模式替代 GNOME 终端
#   用法: ssh abot@IP 'bash ~/3X2KY2/scripts/competition.sh competition_field true'
#   停止: ssh abot@IP 'bash ~/3X2KY2/scripts/competition.sh --stop'
#
# 数据流:
#   game_node --/start--> mission_state_machine  (仅模式1)
#   vlm_node  --/vision_result--> mission_state_machine
#   mission_state_machine --/voiceWords--> doubao_tts → /tts_done
#   mission_state_machine --move_base action--> 导航
#   safety_monitor --/safety_status--> mission_state_machine
#
# 进程管理 (防僵尸进程):
#   - trap EXIT INT TERM: 无论脚本因何退出，都会清理所有子进程
#   - SIGTERM → sleep 3 → SIGKILL → pkill 兜底 (四层保障)
#   - SSH 模式: 内层 setsid 脚本自带 trap，外层可通过 PID 文件精准停止
# ============================================

WS_PATH="${WS_PATH:-${HOME}/3X2KY2}"
PIDFILE=/tmp/abot_competition.pid
INNER_SCRIPT=/tmp/abot_competition_inner.sh

# ============================================
# PID 追踪与清理函数（全局可用）
# ============================================
TRACKED_PIDS=""

track_pid() {
    TRACKED_PIDS="$TRACKED_PIDS $1"
}

cleanup_all() {
    local ts
    ts=$(date '+%H:%M:%S' 2>/dev/null || echo "??:??:??")
    echo "[$ts] [清理] 正在停止所有节点..."

    # L1: SIGTERM 追踪的进程（让 roslaunch 有机会清理子节点）
    for p in $TRACKED_PIDS; do
        kill $p 2>/dev/null
    done

    # L2: 等 3 秒
    sleep 3

    # L3: SIGKILL 残留
    for p in $TRACKED_PIDS; do
        kill -9 $p 2>/dev/null
    done

    # L4: 兜底清扫（防止不属于追踪 PID 树的孤儿进程）
    pkill -9 -f 'roscore'                2>/dev/null || true
    pkill -9 -f 'rosmaster'              2>/dev/null || true
    pkill -9 -f 'rosout'                 2>/dev/null || true
    pkill -9 -f 'roslaunch'              2>/dev/null || true
    pkill -9 -f 'rosrun'                 2>/dev/null || true
    pkill -9 -f 'rplidarNode'            2>/dev/null || true
    pkill -9 -f 'move_base'              2>/dev/null || true
    pkill -9 -f 'amcl|lidar_loc|costmap_cleaner' 2>/dev/null || true
    pkill -9 -f 'mission_state_machine'  2>/dev/null || true
    pkill -9 -f 'safety_monitor'         2>/dev/null || true
    pkill -9 -f 'doubao_tts'             2>/dev/null || true
    pkill -9 -f 'top_view_shot_node'     2>/dev/null || true
    pkill -9 -f 'usb_cam_node'           2>/dev/null || true
    pkill -9 -f 'start_lidar_motor'      2>/dev/null || true
    pkill -9 -f 'lidar_loc'              2>/dev/null || true
    pkill -9 -f 'robot_pose_ekf'         2>/dev/null || true
    pkill -9 -f 'cov_inflate'            2>/dev/null || true
    pkill -9 -f 'cmd_vel_smoother'       2>/dev/null || true
    pkill -9 -f 'cmd_vel_mux'            2>/dev/null || true
    pkill -9 -f 'cmd_vel_safety_guard'   2>/dev/null || true
    pkill -9 -f 'identify_node'          2>/dev/null || true
    pkill -9 -f 'llm_server'             2>/dev/null || true

    rm -f "$PIDFILE"
    echo "[$ts] [清理] 完成"
}

# 注意: trap 在各模式分支中按需注册，避免 SSH 模式下外层脚本退出时误杀内层进程

# ============================================
# --stop: 远端停止（全覆盖 pkill + 端口验证，不依赖 PID 文件）
# ============================================
if [ "${1:-}" = "--stop" ]; then
    echo "=== 停止所有比赛进程 ==="

    # L1: 杀所有内层脚本 (setsid 会话)
    if [ -f "$PIDFILE" ]; then
        PID=$(cat "$PIDFILE" 2>/dev/null)
        [ -n "$PID" ] && kill $PID 2>/dev/null && echo "L1: SIGTERM → PID $PID"
    fi
    pkill -9 -f 'abot_competition_inner\.sh' 2>/dev/null || true

    # L2: 等 1s 让子进程退出
    sleep 1

    # L3: SIGKILL 全部已知 ROS 节点（按进程名/命令行特征）
    pkill -9 -f 'roscore'                 2>/dev/null || true
    pkill -9 -f 'rosmaster'               2>/dev/null || true
    pkill -9 -f 'rosout'                  2>/dev/null || true
    pkill -9 -f 'roslaunch'               2>/dev/null || true
    pkill -9 -f 'rosrun'                  2>/dev/null || true
    pkill -9 -f 'abot_driver'             2>/dev/null || true
    pkill -9 -f 'abot_imu'                2>/dev/null || true
    pkill -9 -f 'rplidarNode'             2>/dev/null || true
    pkill -9 -f 'start_lidar_motor'       2>/dev/null || true
    pkill -9 -f 'laser_filter'            2>/dev/null || true
    pkill -9 -f 'scan_to_scan_filter_chain' 2>/dev/null || true
    pkill -9 -f 'map_server'              2>/dev/null || true
    pkill -9 -f 'amcl\b|lidar_loc|costmap_cleaner' 2>/dev/null || true
    pkill -9 -f 'amcl_tf_bridge'          2>/dev/null || true
    pkill -9 -f 'move_base'               2>/dev/null || true
    pkill -9 -f 'mission_state_machine'   2>/dev/null || true
    pkill -9 -f 'safety_monitor'          2>/dev/null || true
    pkill -9 -f 'doubao_asr'              2>/dev/null || true
    pkill -9 -f 'doubao_tts'              2>/dev/null || true
    pkill -9 -f 'top_view_shot_node'      2>/dev/null || true
    pkill -9 -f 'usb_cam_node'            2>/dev/null || true
    pkill -9 -f 'vlm_node'\n    pkill -9 -f 'doubao_worker'\n    pkill -9 -f '3X2KY2'                2>/dev/null || true
    pkill -9 -f 'identify_node'           2>/dev/null || true
    pkill -9 -f 'llm_server'              2>/dev/null || true
    pkill -9 -f 'robot_pose_ekf'          2>/dev/null || true
    pkill -9 -f 'cov_inflate'            2>/dev/null || true
    pkill -9 -f 'robot_state_publisher'   2>/dev/null || true
    pkill -9 -f 'joint_state_publisher'   2>/dev/null || true
    pkill -9 -f 'imu_filter_madgwick'     2>/dev/null || true
    pkill -9 -f 'odom_ekf'                2>/dev/null || true
    pkill -9 -f 'topic_tools/relay'       2>/dev/null || true
    pkill -9 -f 'wheel_odom_relay'        2>/dev/null || true
    pkill -9 -f 'cmd_vel_mux'             2>/dev/null || true
    pkill -9 -f 'cmd_vel_smoother'        2>/dev/null || true
    pkill -9 -f 'cmd_vel_safety_guard'    2>/dev/null || true
    pkill -9 -f 'rviz'                    2>/dev/null || true
    pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
    pkill -9 -f 'nav_monitor'             2>/dev/null || true
    pkill -9 -f 'tf_echo\|tf2_echo'       2>/dev/null || true
    pkill -9 -f 'rostopic'                2>/dev/null || true
    pkill -9 -f 'rosnode'                 2>/dev/null || true
    pkill -9 -f 'rosservice'              2>/dev/null || true
    pkill -9 -f 'comp_startup\|comp_bringup\|comp_nav\|comp_mission\|comp_vlm\|comp_tts\|comp_wakeup\|comp_relay\|comp_tf_bridge\|comp_initpose\|comp_rviz\|comp_cleanup' 2>/dev/null || true

    rm -f "$PIDFILE" "$INNER_SCRIPT"

    # L4: 等端口释放
    for i in $(seq 1 10); do
        ss -tlnp 2>/dev/null | grep -q ':11311\b' || break
        echo "  等待端口 11311 释放 ($i/10)..."
        sleep 1
    done

    echo "=== 停止完成 ==="
    exit 0
fi

# ============================================
# 参数解析
# ============================================
MAP_NAME="${1:-competition_field}"
# 地图名不带 .yaml 则自动补全 (navigation.launch 的 map_server 需指向 .yaml 文件)
[[ "$MAP_NAME" != *.yaml ]] && MAP_NAME="${MAP_NAME}.yaml"
SIM_MODE="${2:-false}"  # false=模式1(唤醒词)  true=模式3(自动开始)

MODE_NAME="模式1: 完整比赛"
[ "${SIM_MODE}" = "true" ] && MODE_NAME="模式3: 导航+视觉+语音"

echo "========================================"
echo "  ABOT 地面巡航 — ${MODE_NAME}"
echo "========================================"
echo "地图:     ${MAP_NAME}"
echo "节点数:   $([ "${SIM_MODE}" = "true" ] && echo '11 (无唤醒词)' || echo '13')"
echo "PID文件:  ${PIDFILE}"
echo "停止命令: bash ${0} --stop"
echo "========================================"

export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# ============================================
# ROS 网络环境锁定
# ============================================
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# ===== 解释器隔离 (B3/B4) =====
# 不砍 anaconda，只让 /usr/bin 排最前保证 python→py2.7，anaconda site-packages 保留可 import
ENV_PY2='export PATH="/usr/bin:/opt/ros/melodic/bin:$PATH"'
ENV_PY39='export PATH="/opt/ros/melodic/bin:$(echo "$PATH" | sed -e "s#/home/abot/anaconda3[^:]*:##g" -e "s#:/home/abot/anaconda3[^:]*##g")"
__PY39SHIM=/tmp/abot_py39_shim; mkdir -p "$__PY39SHIM"; ln -sf /home/abot/anaconda3/envs/py39/bin/python3.9 "$__PY39SHIM/python3"; export PATH="$__PY39SHIM:$PATH"'

READY_HELPERS='
wait_master() { for i in $(seq 1 40); do timeout 2 rostopic list >/dev/null 2>&1 && return 0; sleep 1; done; echo "[warn] roscore 未就绪, 继续"; }
wait_topic() { for i in $(seq 1 ${2:-40}); do timeout 2 rostopic list 2>/dev/null | grep -qx "$1" && return 0; sleep 1; done; echo "[warn] 等待 $1 超时, 继续"; }
wait_node() { for i in $(seq 1 ${2:-40}); do timeout 2 rosnode list 2>/dev/null | grep -qx "$1" && return 0; sleep 1; done; echo "[warn] 等待节点 $1 超时, 继续"; }
'

# ============================================
# 环境检测
# ============================================
if grep -qi microsoft /proc/version 2>/dev/null; then
    # ============================================
    # WSL 仿真模式
    # ============================================
    trap cleanup_all EXIT INT TERM

    echo "[WSL] 仿真后台启动..."

    roscore &
    track_pid $!
    sleep 2

    source /opt/ros/melodic/setup.bash
    source ${WS_PATH}/devel/setup.bash

    # 1. 底盘 + 传感器层
    echo "[1/5] 启动底盘驱动..."
    roslaunch abot_bringup robot_with_imu.launch &
    track_pid $!
    sleep 5

    # 2. 导航层
    echo "[2/5] 启动导航栈..."
    roslaunch robot_slam navigation.launch map_name:=${MAP_NAME} &
    track_pid $!
    sleep 8

    # 3. 唤醒词检测
    echo "[3/5] 启动唤醒词检测..."
    roslaunch robot_slam GameStart.launch &
    track_pid $!
    sleep 2

    # 4. VLM 图像识别
    echo "[4/5] 启动 VLM 视觉识别..."
    roslaunch abot_vlm vlm_node.launch &
    track_pid $!
    sleep 2

    # 5. 任务状态机 + 安全监控
    echo "[5/5] 启动任务状态机..."
    roslaunch mission_manager sim_mission.launch sim_mode:=${SIM_MODE} &
    track_pid $!
    sleep 2

    echo "========================================"
    echo "  全部 14 个节点已启动"
    echo "========================================"
    echo ""
    echo "节点清单:"
    echo "  [底层] abot_driver, abot_imu, rplidar, box_filter"
    echo "  [融合] robot_state_publisher, robot_pose_ekf"
    echo "  [定位] map_server, lidar_loc"
    echo "  [导航] move_base (GlobalPlanner + DWA)"
    echo "  [语音] game_node (Snowboy)"
    echo "  [视觉] vlm_node (豆包 Vision Pro)"
    echo "  [任务] mission_state_machine, safety_monitor"
    echo ""
    if [ "${SIM_MODE}" = "true" ]; then
        echo "仿真模式：发布 'sim_wakeup' 到 /start topic 触发比赛开始"
        echo "  rostopic pub /start std_msgs/String \"data: 'sim_wakeup'\""
    fi
    echo ""
    echo "Ctrl+C 停止所有节点"

    wait  # 等待所有后台进程; Ctrl+C → SIGINT → cleanup_all

elif [ -n "$SSH_CONNECTION" ] || ! command -v gnome-terminal >/dev/null 2>&1; then
    # ============================================
    # SSH 后台模式 (无 GNOME 桌面)
    #
    # 架构：外层脚本写内层脚本到磁盘 → setsid 执行内层脚本 → 外层退出
    # 内层脚本自带 trap，收到 SIGTERM 时清理所有节点
    # 停止方式: bash competition.sh --stop (读取 PID 文件 kill setsid 会话)
    # ============================================

    # --- 外层陷阱：仅处理 Ctrl+C (SIGINT)，不处理 EXIT ---
    # SIGINT: 用户在外层 sleep 期间按了 Ctrl+C → 杀 setsid 会话
    # EXIT: 外层正常退出 → 不触发清理（内层脚本负责）
    cleanup_outer() {
        echo ""
        echo "[外层] 收到中断信号，停止竞赛会话..."
        if [ -f "$PIDFILE" ]; then
            kill $(cat "$PIDFILE") 2>/dev/null || true
        fi
        # 短时等待内层 cleanup 生效
        sleep 2
        cleanup_all
    }
    # 注意: 不 trap SIGHUP — SSH 断连不应杀 setsid 会话（用户可能重连后用 --stop 管理）
    trap cleanup_outer INT TERM

    echo "[SSH] 清理旧进程..."
    pkill -f 'roscore|roslaunch|rosrun|rplidarNode|move_base|amcl|lidar_loc|costmap_cleaner|mission_state_machine|safety_monitor|doubao_tts|top_view_shot_node|usb_cam_node' 2>/dev/null || true
    sleep 3
    # 确保 ROS master 端口释放 (lsof 可能不存在，用 ss 替代)
    while ss -tlnp 2>/dev/null | grep -q ':11311\b'; do
        echo "  等待端口 11311 释放..."
        sleep 1
    done
    rm -f /tmp/comp_*.log "$INNER_SCRIPT" "$PIDFILE"

    # --- 写入内层脚本（用 quoted heredoc 防外层变量展开） ---
    cat > "$INNER_SCRIPT" << 'INNER_SCRIPT_END'
#!/bin/bash
# ============================================
# 竞赛内层脚本 — 运行在 setsid 会话中
# 自带 trap: 收到 SIGTERM 时清理所有节点
# 参数: $1=MAP_NAME  $2=SIM_MODE  $3=WS_PATH
# ============================================
MAP_NAME="$1"
SIM_MODE="$2"
WS_PATH="$3"
PIDFILE=/tmp/abot_competition.pid
echo $$ > "$PIDFILE"

PIDS=""
track() { PIDS="$PIDS $1"; }

# ---------- 内层清理 ----------
inner_cleanup() {
    local ts
    ts=$(date '+%H:%M:%S' 2>/dev/null || echo "??:??:??")
    echo "[$ts] [内层] 收到退出信号，清理所有节点..." | tee -a /tmp/comp_cleanup.log

    # L1: SIGTERM 所有追踪进程
    for p in $PIDS; do
        kill $p 2>/dev/null
    done

    # L2: 等 3 秒
    sleep 3

    # L3: SIGKILL 残留
    for p in $PIDS; do
        kill -9 $p 2>/dev/null
    done

    # L4: 兜底（与外层 cleanup_all 保持同步）
    pkill -9 -f 'roscore'                2>/dev/null || true
    pkill -9 -f 'rosmaster'              2>/dev/null || true
    pkill -9 -f 'rosout'                 2>/dev/null || true
    pkill -9 -f 'roslaunch'              2>/dev/null || true
    pkill -9 -f 'rosrun'                 2>/dev/null || true
    pkill -9 -f 'rplidarNode'            2>/dev/null || true
    pkill -9 -f 'move_base'              2>/dev/null || true
    pkill -9 -f 'amcl|lidar_loc|costmap_cleaner' 2>/dev/null || true
    pkill -9 -f 'mission_state_machine'  2>/dev/null || true
    pkill -9 -f 'safety_monitor'         2>/dev/null || true
    pkill -9 -f 'doubao_tts'             2>/dev/null || true
    pkill -9 -f 'top_view_shot_node'     2>/dev/null || true
    pkill -9 -f 'usb_cam_node'           2>/dev/null || true
    pkill -9 -f 'start_lidar_motor'      2>/dev/null || true
    pkill -9 -f 'identify_node'          2>/dev/null || true
    pkill -9 -f 'llm_server'             2>/dev/null || true
    pkill -9 -f 'robot_pose_ekf'         2>/dev/null || true
    pkill -9 -f 'cov_inflate'            2>/dev/null || true
    pkill -9 -f 'cmd_vel_smoother'       2>/dev/null || true
    pkill -9 -f 'cmd_vel_mux'            2>/dev/null || true
    pkill -9 -f 'cmd_vel_safety_guard'   2>/dev/null || true

    rm -f "$PIDFILE"
    echo "[$ts] [内层] 清理完成" | tee -a /tmp/comp_cleanup.log
}
trap inner_cleanup EXIT INT TERM

# ---------- PATH 设置 ----------
export PATH="/usr/bin:/opt/ros/melodic/bin:/home/abot/anaconda3/envs/py39/bin:$PATH"
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

source /opt/ros/melodic/setup.bash
source "${WS_PATH}/devel/setup.bash"

echo "[内层] === 竞赛启动 $(date '+%H:%M:%S') ==="
echo "[内层] 地图: ${MAP_NAME}  模式: ${SIM_MODE}"

# [1] roscore
echo '[1/5] roscore...'
roscore > /tmp/comp_roscore.log 2>&1 &
track $!
sleep 5

# [2] 底盘驱动 (IMU + LiDAR + EKF)
echo '[2/5] 底盘驱动...'
roslaunch abot_bringup robot_with_imu.launch > /tmp/comp_bringup.log 2>&1 &
track $!
sleep 12

# [3] 导航栈 (map_server + lidar_loc + move_base)
echo '[3/5] 导航栈...'
roslaunch robot_slam navigation.launch map_name:=${MAP_NAME} \
    initial_pose_x:=-1.6 initial_pose_y:=1.6 initial_pose_a:=1.5708 \
    > /tmp/comp_nav.log 2>&1 &
track $!
sleep 15

# [3.5] 里程计: robot_with_imu.launch 中的 odom_ekf 已将
# /robot_pose_ekf/odom_combined (EKF 融合 IMU+轮式) 转换成 /odom (Odometry)。
# 不再 relay 裸轮式 /wheel_odom（麦克纳姆轮滑移导致累积漂移）。
echo '[3.5] 里程计: odom_ekf 已发布 EKF 融合 /odom（跳过裸轮式 relay）'

# [3.55] 定位 TF：jie_ware/lidar_loc 直接发布 map->odom，不再启动 amcl_tf_bridge
echo '[3.55] 定位 TF: lidar_loc 发布 map->odom'
sleep 2

# [3.6] 初始位姿 (比赛场地起点)
echo '[3.6] 发送初始位姿...'
sleep 1
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: -1.6, y: 1.6, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}, covariance: [0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.017]}}" \
    > /tmp/comp_initpose.log 2>&1 || true
# 短等定位 TF（lidar_loc 收到 /initialpose、/map、/scan_filtered 后开始发布 map→odom）
sleep 3
for i in $(seq 1 3); do
    timeout 2 rosrun tf tf_echo map odom > /dev/null 2>&1 && break
    sleep 1
done

# [4] VLM + TTS
echo '[4/5] VLM + TTS...'
roslaunch abot_vlm vlm_node.launch > /tmp/comp_vlm.log 2>&1 &
track $!
sleep 3
rosrun robot_slam doubao_tts.py > /tmp/comp_tts.log 2>&1 &
track $!
sleep 2

# [5] 状态机 + 安全监控（必须在 ASR 之前启动，确保 /start 订阅者就绪）
echo '[5/7] 状态机 + 安全...'
roslaunch mission_manager sim_mission.launch sim_mode:=${SIM_MODE} > /tmp/comp_mission.log 2>&1 &
track $!
sleep 3

# [6] 语音唤醒（仅非 sim 模式，豆包 ASR 检测"开始比赛"）
if [ "${SIM_MODE}" = "false" ]; then
    echo '[6/7] 语音唤醒 (豆包 ASR)...'
    # 启动豆包 ASR（需 py3.9 shim），提示音由 ASR 节点内部播放
    __PY39SHIM=/tmp/abot_py39_shim
    mkdir -p "$__PY39SHIM"
    ln -sf /home/abot/anaconda3/envs/py39/bin/python3.9 "$__PY39SHIM/python3"
    PATH="$__PY39SHIM:$PATH" rosrun robot_slam doubao_asr.py \
        > /tmp/comp_wakeup_asr.log 2>&1 &
    track $!
    sleep 3
fi

# [7] RViz 可视化
echo '[7] RViz...'
roslaunch robot_slam view_nav.launch > /tmp/comp_rviz.log 2>&1 &
track $!
sleep 2

echo "=== 全部启动完成 ($(date '+%H:%M:%S')) ==="
timeout 3 rostopic list 2>/dev/null | wc -l | xargs -I{} echo "话题数: {}"
echo "日志: /tmp/comp_*.log"
echo "停止: bash ${WS_PATH}/scripts/competition.sh --stop"

# 持续运行直到收到退出信号
wait
INNER_SCRIPT_END

    chmod +x "$INNER_SCRIPT"

    echo "[SSH] 后台启动 (模式: ${MODE_NAME})..."
    setsid bash "$INNER_SCRIPT" "$MAP_NAME" "$SIM_MODE" "$WS_PATH" > /tmp/comp_startup.log 2>&1 &
    INNER_PID=$!
    disown

    echo "后台会话已启动 (PID: $INNER_PID)"
    echo "内层脚本: $INNER_SCRIPT"
    sleep 35

    # 取消外层 trap（让脚本可以安全退出）
    trap - INT TERM

    source /opt/ros/melodic/setup.bash
    source ${WS_PATH}/devel/setup.bash
    echo ""
    echo "=== 节点 ==="
    timeout 3 rosnode list 2>/dev/null || echo "(等待中...)"
    echo ""
    echo "=== 关键话题 ==="
    timeout 3 rostopic list 2>/dev/null | grep -E "/scan_filtered|/tf\b|/map\b|/vision_result|/voiceWords|/tts_done|/mission_state" || echo "(等待中...)"
    echo ""
    echo "========================================"
    echo "  竞赛已后台运行"
    echo "  停止命令: ssh abot@$(hostname -I 2>/dev/null | awk '{print $1}') 'bash ${WS_PATH}/scripts/competition.sh --stop'"
    echo "  监控命令: ssh abot@$(hostname -I 2>/dev/null | awk '{print $1}') 'source /opt/ros/melodic/setup.bash && source ${WS_PATH}/devel/setup.bash && rosrun robot_slam nav_monitor.py'"
    echo "========================================"
    [ "${SIM_MODE}" = "false" ] && echo "说出'开始比赛'启动..." || echo "模式3: 5s 后自动开始. 监控日志: /tmp/comp_mission.log"

else
    # ============================================
    # GNOME 桌面模式 (原有逻辑 + trap)
    # ============================================
    trap cleanup_all EXIT INT TERM

    # 窗口 1: roscore
    gnome-terminal -- bash -c '
        source /opt/ros/melodic/setup.bash
        roscore
        exec bash' &
    track_pid $!
    sleep 1

    # 窗口 2: 底盘 + IMU + LiDAR + EKF + 模型
    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        $ENV_PY2
        $READY_HELPERS
        wait_master
        roslaunch abot_bringup robot_with_imu.launch
        exec bash" &
    track_pid $!
    sleep 1

    # 窗口 3: 导航栈
    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        $ENV_PY2
        $READY_HELPERS
        wait_master
        wait_topic /scan_filtered 40
        roslaunch robot_slam navigation.launch map_name:=${MAP_NAME}
        exec bash" &
    track_pid $!
    sleep 1

    # 窗口 4: 状态机 + 安全（必须在 ASR 前启动，确保 /start 订阅者就绪）
    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        $ENV_PY2
        $READY_HELPERS
        wait_master
        wait_topic /move_base/status 60
        roslaunch mission_manager sim_mission.launch sim_mode:=${SIM_MODE}
        exec bash" &
    track_pid $!
    sleep 1

    # 窗口 5: ASR + VLM + TTS
    if [ "${SIM_MODE}" = "false" ]; then
        gnome-terminal -- bash -c "
            source /opt/ros/melodic/setup.bash
            source ${WS_PATH}/devel/setup.bash
            $ENV_PY39
            $READY_HELPERS
            wait_master
            wait_topic /mission_heartbeat 30
            rosrun robot_slam doubao_tts.py &
            sleep 2
            roslaunch abot_vlm vlm_node.launch &
            sleep 2
            rosrun robot_slam doubao_asr.py &
            exec bash" &
    else
        gnome-terminal -- bash -c "
            source /opt/ros/melodic/setup.bash
            source ${WS_PATH}/devel/setup.bash
            $ENV_PY39
            $READY_HELPERS
            wait_master
            roslaunch abot_vlm vlm_node.launch &
            sleep 2
            rosrun robot_slam doubao_tts.py &
            exec bash" &
    fi
    track_pid $!
    sleep 1

    # 窗口 6: RViz
    gnome-terminal -- bash -c "
        source /opt/ros/melodic/setup.bash
        source ${WS_PATH}/devel/setup.bash
        $ENV_PY2
        $READY_HELPERS
        wait_master
        wait_topic /map 30
        roslaunch robot_slam view_nav.launch
        exec bash" &
    track_pid $!
    sleep 1

    echo "========================================"
    echo "  6 个终端窗口已启动"
    echo "========================================"
    echo "窗1: roscore"
    echo "窗2: 底盘驱动 (abot_driver + IMU + LiDAR + EKF)"
    echo "窗3: 导航栈 (map_server + lidar_loc + move_base)"
    echo "窗4: $([ "${SIM_MODE}" = "false" ] && echo '豆包 ASR 语音识别 + ')/VLM 视觉 + TTS 语音"
    echo "窗5: 任务状态机 + 安全监控"
    echo "窗6: RViz 可视化"
    echo ""
    echo "关闭所有 GNOME 终端窗口即停止全部节点"
    echo "或 Ctrl+C 触发全局清理"
    [ "${SIM_MODE}" = "false" ] && echo "说出'开始比赛'启动..." || echo "模式3: 5s 后自动开始"

    echo "按 Enter 停止所有节点..."
    read _line
    cleanup_all
fi

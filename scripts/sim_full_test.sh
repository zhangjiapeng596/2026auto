#!/bin/bash
# WSL 完整比赛仿真测试
# 用法: wsl bash scripts/sim_full_test.sh
#
# 进程管理:
#   trap EXIT INT TERM 防僵尸进程 — 中断或异常退出时自动清理 roslaunch
set -eu
set -o pipefail

WS="${WS_PATH:-$HOME/3X2KY2}"
LOG="/tmp/sim_full.log"
SRC="${SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PIDFILE="/tmp/sim_full.pid"

# ============================================
# 清理函数（四层保障: SIGTERM → sleep → SIGKILL → pkill）
# ============================================
cleanup() {
    echo ""
    echo "[清理] 停止仿真..."

    # 先杀 roslaunch（会级联清理其子节点）
    if [ -n "${SIM_PID:-}" ]; then
        kill $SIM_PID 2>/dev/null || true
    fi

    # 等 roslaunch 清理
    sleep 3

    # SIGKILL 残留
    if [ -n "${SIM_PID:-}" ]; then
        kill -9 $SIM_PID 2>/dev/null || true
    fi

    # 兜底清扫
    killall -9 rosmaster rosout roscore 2>/dev/null || true
    killall -9 roslaunch rviz move_base amcl map_server 2>/dev/null || true
    killall -9 sim_robot robot_state_publisher mock_vlm mock_tts 2>/dev/null || true
    killall -9 mission_state_machine safety_monitor cartographer_node 2>/dev/null || true

    rm -f "$PIDFILE"
    echo "[清理] 完成"
}
trap cleanup EXIT INT TERM

echo "$$" > "$PIDFILE"

# ROS setup 脚本会读取未定义变量，source 时临时关闭 nounset。
set +u
source /opt/ros/melodic/setup.bash
source "$WS"/devel/setup.bash
set -u

# 从 mission.yaml 读取地图名
MAP_NAME=$(python -c "import yaml; print(yaml.safe_load(open('$SRC/config/mission.yaml'))['mission']['map_name'])" 2>/dev/null || echo "competition_field")
echo "=== ABOT 地面巡航 完整仿真 (地图: $MAP_NAME) ==="
echo "Ctrl+C 停止并清理"
START=$(date +%s)

# 1. 清理
echo "[1/4] 清理旧进程..."
killall -9 rosmaster rosout roscore roslaunch rviz move_base amcl map_server sim_robot robot_state_publisher mock_vlm mock_tts mission_state_machine safety_monitor cartographer_node 2>/dev/null || true
sleep 2

# 2. 同步 (Windows 源码 → WSL 工作空间) — 递归覆盖，新增目录/文件自动包含
echo "[2/4] 同步源码..."
# 清理源+目标的 .pyc / __pycache__ (避免 cp 权限和 CRLF 校验错误)
find "$SRC"/src/ "$SRC"/config/ "$WS"/src/ "$WS"/config/ \
    \( -name '*.pyc' -o -name '__pycache__' \) -exec rm -rf {} + 2>/dev/null || true
# 递归同步
cp -r "$SRC"/src/mission_manager/. "$WS"/src/mission_manager/ 2>/dev/null || true
cp -r "$SRC"/src/common/.          "$WS"/src/common/           2>/dev/null || true
cp -r "$SRC"/config/.              "$WS"/config/               2>/dev/null || true
cp -r "$SRC"/src/robot_slam/.      "$WS"/src/robot_slam/       2>/dev/null || true
# CRLF → LF: 所有文本文件去 \r (Windows git → Linux 内核/roslaunch)
find "$WS"/src/ "$WS"/config/ -type f \
    \( -name '*.py' -o -name '*.launch' -o -name '*.xml' -o -name '*.yaml' \
       -o -name '*.rviz' -o -name '*.sh' -o -name '*.lua' -o -name '*.urdf' \
       -o -name '*.cfg' -o -name '*.md' \) \
    -exec sed -i 's/\r$//' {} + 2>/dev/null || true
echo "  地图: $MAP_NAME  ($(head -1 "$WS"/src/robot_slam/maps/"$MAP_NAME".yaml 2>/dev/null || echo 'MISSING!'))"

# 3. 启动
echo "[3/4] 启动仿真 (map_name=$MAP_NAME)..."
roslaunch mission_manager sim_full_mission.launch map_name:="$MAP_NAME" > "$LOG" 2>&1 &
SIM_PID=$!

# 等日志出现
for i in $(seq 1 15); do
    [ -s "$LOG" ] && break
    sleep 1
done

# 等待仿真完成（最多 180s 比赛时间 + 启动 buffer）
echo "  等待仿真运行..."
TIMEOUT=200
FINISHED=false
for i in $(seq 1 $TIMEOUT); do
    sleep 1
    if grep -qE "DONE|ABORT" "$LOG" 2>/dev/null; then
        echo "  任务结束 (T+${i}s)"
        FINISHED=true
        break
    fi
    # 检查 roslaunch 是否还在运行（崩溃检测）
    if ! kill -0 $SIM_PID 2>/dev/null; then
        echo "  roslaunch 已退出 (T+${i}s)"
        FINISHED=true
        break
    fi
done

if [ "$FINISHED" != "true" ]; then
    echo "  超时 (${TIMEOUT}s)，强制停止"
fi

# 4. 报告
echo ""
echo "=== 仿真报告 (总耗时 $(($(date +%s) - START))s) ==="
echo "--- 任务日志 ---"
strings "$LOG" 2>/dev/null | grep -iE -- "->|Navigating|Arrived|Recogni|DONE|ABORT|TTS|Phase" | tail -20
echo ""
echo "--- 错误 ---"
strings "$LOG" 2>/dev/null | grep -iE "ERROR|FATAL|Traceback" | grep -v "Unable" | tail -10 || echo "(无)"
echo ""
echo "完整日志: $LOG"

# 清理由 trap 自动执行
echo "已停止"

#!/bin/bash
# RViz manual navigation debug launcher.
# Starts only bringup + lidar localization + move_base + RViz.
# No mission state machine, ASR, VLM, or TTS is started.

set -u

WS_PATH="${WS_PATH:-${HOME}/3X2KY2}"
MAP_NAME="${1:-competition_field}"
[[ "$MAP_NAME" != *.yaml ]] && MAP_NAME="${MAP_NAME}.yaml"
PIDFILE=/tmp/abot_debug_nav.pid
LOG_DIR=/tmp/abot_debug_nav_logs

mkdir -p "$LOG_DIR"
cd "$WS_PATH" || exit 1

echo "========================================"
echo "  ABOT RViz manual navigation debug"
echo "========================================"
echo "Map: $MAP_NAME"
echo "Starts: bringup, lidar, localization, move_base, RViz"
echo "Skips: mission state machine, ASR, VLM, TTS"
echo "Stop: press Enter or run bash ${WS_PATH}/scripts/competition.sh --stop"
echo "========================================"

bash "$WS_PATH/scripts/stop.sh" >/tmp/abot_debug_nav_stop.log 2>&1 || true
sleep 1

source /opt/ros/melodic/setup.bash
source "$WS_PATH/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}

PIDS=""
track() { PIDS="$PIDS $1"; }

cleanup() {
    trap - EXIT INT TERM
    echo "[debug_nav] cleaning debug nodes..."
    bash "$WS_PATH/scripts/stop.sh"
}
trap cleanup EXIT INT TERM

wait_master() {
    for _i in $(seq 1 40); do
        timeout 2 rostopic list >/dev/null 2>&1 && return 0
        sleep 1
    done
    echo "[warn] roscore is not ready"
    return 1
}

wait_topic() {
    local topic="$1"
    local tries="${2:-40}"
    for _i in $(seq 1 "$tries"); do
        timeout 2 rostopic list 2>/dev/null | grep -qx "$topic" && return 0
        sleep 1
    done
    echo "[warn] timed out waiting for $topic, continuing"
    return 1
}

roscore >"$LOG_DIR/roscore.log" 2>&1 &
track $!
wait_master

roslaunch abot_bringup robot_with_imu.launch >"$LOG_DIR/bringup.log" 2>&1 &
track $!
wait_topic /scan_filtered 50

roslaunch robot_slam navigation.launch map_name:="$MAP_NAME" >"$LOG_DIR/navigation.log" 2>&1 &
track $!
wait_topic /move_base/status 70

roslaunch robot_slam view_nav.launch >"$LOG_DIR/rviz.log" 2>&1 &
track $!

echo $$ > "$PIDFILE"

echo "========================================"
echo "  Debug environment is running"
echo "========================================"
echo "RViz:"
echo "  1. Use 2D Pose Estimate to reset initial pose"
echo "  2. Use 2D Nav Goal to send a manual goal"
echo "Useful topics: /scan_filtered, /map, /move_base/global_costmap/costmap"
echo "Logs: $LOG_DIR"
echo "Press Enter to stop and clean all nodes..."
read _line
cleanup

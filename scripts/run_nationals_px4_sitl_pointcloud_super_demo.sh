#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${WORKSPACE:-${HOME}/super_ws}}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${REPO:-${SUPER_WS}/src/SUPER_DRONE}}"
GEZOGO_DIR="${GEZOGO_DIR:-${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}}"
SEED="${SEED:-2026}"
HEADLESS="${HEADLESS:-1}"
SITL_RECORD_BAG="${SITL_RECORD_BAG:-0}"
GUI_PREPARE_WAIT_SEC="${GUI_PREPARE_WAIT_SEC:-45}"
GUI_HOLD_AFTER_PASS_SEC="${GUI_HOLD_AFTER_PASS_SEC:-5}"
TIMEOUT_SEC="${TIMEOUT_SEC:-240}"
ROS_PORT="${ROS_PORT:-11325}"
GAZEBO_PORT="${GAZEBO_PORT:-11347}"
TARGET_ALT="${SITL_SUPER_TARGET_ALT:-1.0}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${LAYOUT_PATH:-${GENERATED_DIR}/layout.json}"
LOG_DIR="${SUPER_DRONE_DIR}/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/nationals_px4_sitl_pointcloud_super_demo_${STAMP}.log"
BAG_PATH="${LOG_DIR}/nationals_px4_sitl_pointcloud_super_demo_${STAMP}.bag"
SUMMARY_PATH="${LOG_DIR}/nationals_px4_sitl_pointcloud_super_demo_${STAMP}.summary.json"

export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/super_drone_roslog}"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
mkdir -p "${LOG_DIR}" "${ROS_LOG_DIR}" "${ROS_HOME}"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "FAIL: /opt/ros/noetic/setup.bash not found" >&2
    exit 1
fi
if [ ! -f "${SUPER_WS}/devel/setup.bash" ]; then
    echo "FAIL: ${SUPER_WS}/devel/setup.bash not found. Run catkin_make first." >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${SUPER_WS}/devel/setup.bash"
set -u

for package in mavros px4ctrl quadrotor_msgs mission_planner super_planner; do
    if ! rospack find "${package}" >/dev/null 2>&1; then
        echo "FAIL: ROS package ${package} is not found. Run catkin_make/preflight first." >&2
        exit 1
    fi
done

PIDS=()
cleanup() {
    set +e
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            kill -TERM "-${pid}" >/dev/null 2>&1 || kill -TERM "${pid}" >/dev/null 2>&1 || true
        fi
    done
    sleep 2
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            kill -KILL "-${pid}" >/dev/null 2>&1 || kill -KILL "${pid}" >/dev/null 2>&1 || true
        fi
        wait "${pid}" >/dev/null 2>&1 || true
    done
}
trap cleanup EXIT

run_bg() {
    local name="$1"
    shift
    echo "[pointcloud_super] starting ${name}" | tee -a "${LOG_FILE}"
    if [ "${name}" = "px4_sitl" ]; then
        setsid "$@" >/dev/null 2>&1 &
    else
        setsid "$@" >> "${LOG_FILE}" 2>&1 &
    fi
    PIDS+=("$!")
}

wait_topic() {
    local topic="$1"
    local timeout_s="$2"
    local start now
    start="$(date +%s)"
    while true; do
        if timeout 3 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
            echo "[pointcloud_super] topic ready: ${topic}" | tee -a "${LOG_FILE}"
            return 0
        fi
        now="$(date +%s)"
        if [ "$((now - start))" -ge "${timeout_s}" ]; then
            echo "FAIL: timeout waiting for ${topic}" | tee -a "${LOG_FILE}"
            return 1
        fi
        sleep 1
    done
}

wait_grep() {
    local label="$1"
    local timeout_s="$2"
    shift 2
    local start now
    start="$(date +%s)"
    while true; do
        if "$@" >/dev/null 2>&1; then
            echo "[pointcloud_super] ready: ${label}" | tee -a "${LOG_FILE}"
            return 0
        fi
        now="$(date +%s)"
        if [ "$((now - start))" -ge "${timeout_s}" ]; then
            echo "FAIL: timeout waiting for ${label}" | tee -a "${LOG_FILE}"
            return 1
        fi
        sleep 1
    done
}

wait_gzserver_pid() {
    local timeout_s="$1"
    local start now pid
    start="$(date +%s)"
    while true; do
        pid="$(pgrep -n -x gzserver 2>/dev/null || true)"
        if [ -n "${pid}" ]; then
            echo "${pid}"
            return 0
        fi
        now="$(date +%s)"
        if [ "$((now - start))" -ge "${timeout_s}" ]; then
            return 1
        fi
        sleep 1
    done
}

gazebo_master_uri_from_pid() {
    local pid="$1"
    local uri=""
    if [ -r "/proc/${pid}/environ" ]; then
        uri="$(tr '\0' '\n' < "/proc/${pid}/environ" | sed -n 's/^GAZEBO_MASTER_URI=//p' | tail -n 1 || true)"
    fi
    if [ -z "${uri}" ]; then
        uri="${GAZEBO_MASTER_URI:-}"
    fi
    if [ -z "${uri}" ]; then
        uri="http://localhost:11345"
    fi
    echo "${uri}"
}

start_gazebo_gui_if_requested() {
    if [ "${HEADLESS}" != "0" ]; then
        echo "[gui] HEADLESS=${HEADLESS}; not starting gzclient" | tee -a "${LOG_FILE}"
        return 0
    fi
    local gzserver_pid gui_master_uri gzclient_pid
    if ! gzserver_pid="$(wait_gzserver_pid 60)"; then
        echo "FAIL: timeout waiting for gzserver before starting gzclient" | tee -a "${LOG_FILE}"
        return 1
    fi
    gui_master_uri="$(gazebo_master_uri_from_pid "${gzserver_pid}")"
    echo "[gui] starting gzclient" | tee -a "${LOG_FILE}"
    echo "[gui] GAZEBO_MASTER_URI=${gui_master_uri}" | tee -a "${LOG_FILE}"
    setsid nice -n 15 env GAZEBO_MASTER_URI="${gui_master_uri}" gzclient >> "${LOG_FILE}" 2>&1 &
    gzclient_pid="$!"
    PIDS+=("${gzclient_pid}")
    echo "[gui] gzclient_pid=${gzclient_pid}" | tee -a "${LOG_FILE}"
    sleep 2
    if ! kill -0 "${gzclient_pid}" >/dev/null 2>&1; then
        echo "FAIL: gzclient exited before Gazebo GUI preload" | tee -a "${LOG_FILE}"
        return 1
    fi
    ps aux | grep '[g]zclient' | tee -a "${LOG_FILE}" || true
}

wait_gazebo_gui_prepare_if_requested() {
    if [ "${HEADLESS}" != "0" ]; then
        return 0
    fi
    echo "[gui] waiting ${GUI_PREPARE_WAIT_SEC}s for Gazebo world to render" | tee -a "${LOG_FILE}"
    sleep "${GUI_PREPARE_WAIT_SEC}"
}

hold_gazebo_gui_after_pass_if_requested() {
    if [ "${HEADLESS}" != "0" ]; then
        return 0
    fi
    echo "[gui] demo PASS, holding Gazebo window for ${GUI_HOLD_AFTER_PASS_SEC}s" | tee -a "${LOG_FILE}"
    sleep "${GUI_HOLD_AFTER_PASS_SEC}"
}

wait_static_odom_for_takeoff() {
    local timeout_s="${SITL_SUPER_STATIC_ODOM_WAIT_SEC:-30}"
    local vel_max="${SITL_SUPER_STATIC_ODOM_VEL_MAX:-0.095}"
    echo "[pointcloud_super] waiting for static odom before TAKEOFF vel<=${vel_max}m/s timeout=${timeout_s}s" | tee -a "${LOG_FILE}"
    python3 - "${timeout_s}" "${vel_max}" 2>&1 <<'PY' | tee -a "${LOG_FILE}"
import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry

timeout_s = float(sys.argv[1])
vel_max = float(sys.argv[2])
latest = None

def cb(msg):
    global latest
    latest = msg

rospy.init_node("nationals_pointcloud_super_static_odom_wait", anonymous=True, disable_signals=True)
rospy.Subscriber("/mavros/local_position/odom", Odometry, cb, queue_size=10)
deadline = time.monotonic() + timeout_s
stable_since = None
last_norm = float("nan")
rate = rospy.Rate(20)
while not rospy.is_shutdown() and time.monotonic() < deadline:
    if latest is not None:
        v = latest.twist.twist.linear
        last_norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        now = time.monotonic()
        if last_norm <= vel_max:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= 0.5:
                print(f"[pointcloud_super] static odom ready vel={last_norm:.4f}m/s")
                sys.exit(0)
        else:
            stable_since = None
    rate.sleep()
print(f"FAIL: static odom timeout last_vel={last_norm:.4f}m/s threshold={vel_max:.4f}m/s")
sys.exit(1)
PY
}

echo "[pointcloud_super] SUPER pointcloud PX4 SITL demo; validator does not publish /position_cmd" | tee -a "${LOG_FILE}"
echo "[pointcloud_super] summary: ${SUMMARY_PATH}" | tee -a "${LOG_FILE}"
echo "[pointcloud_super] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[pointcloud_super] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if ! rostopic list >/dev/null 2>&1; then
    run_bg roscore roscore -p "${ROS_PORT}"
    sleep 3
fi

run_bg px4_sitl env HEADLESS="${HEADLESS}" SUPER_WS="${SUPER_WS}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4_sitl_world.sh"
start_gazebo_gui_if_requested
sleep 10

run_bg mavros env SUPER_WS="${SUPER_WS}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_mavros.sh"
wait_topic /mavros/state 45
wait_grep "MAVROS connected=True" 45 bash -lc "rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'"

run_bg odom_relay env ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" rosrun topic_tools relay /mavros/local_position/odom /Odom_high_freq __name:=nationals_pointcloud_super_odom_relay
wait_topic /Odom_high_freq 20

run_bg super env SUPER_WS="${SUPER_WS}" SUPER_DRONE_DIR="${SUPER_DRONE_DIR}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" LAYOUT_PATH="${LAYOUT_PATH}" SITL_START_MISSION=false SITL_START_SMOKE_GOAL=false SITL_PLANNER_CONFIG_NAME="${SITL_PLANNER_CONFIG_NAME:-super_drone_px4_sitl_smoke.yaml}" SITL_INCLUDE_BOUNDARY_WALLS="${SITL_INCLUDE_BOUNDARY_WALLS:-true}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_super_sitl_smoke.sh"
wait_topic /cloud_registered 45
wait_grep "fsm_node running" 45 bash -lc "rosnode list 2>/dev/null | grep -q '^/fsm_node$'"
wait_gazebo_gui_prepare_if_requested

run_bg px4ctrl env SUPER_WS="${SUPER_WS}" SITL_TAKEOFF_HEIGHT="${TARGET_ALT}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4ctrl_sitl.sh"
wait_topic /mavros/setpoint_raw/attitude 45
wait_grep "px4ctrl subscribes /position_cmd" 45 bash -lc "rostopic info /position_cmd 2>/dev/null | grep -q '/px4ctrl'"
wait_static_odom_for_takeoff

if [ "${SITL_RECORD_BAG}" = "1" ]; then
    rosbag record -O "${BAG_PATH}" --lz4 \
        /cloud_registered \
        /Odom_high_freq \
        /planning/click_goal \
        /position_cmd \
        /mavros/state \
        /mavros/local_position/odom \
        /mavros/setpoint_raw/attitude \
        /px4ctrl/takeoff_land \
        /rosout_agg >> "${LOG_FILE}" 2>&1 &
    PIDS+=("$!")
    sleep 1
else
    echo "[pointcloud_super] bag recording disabled by SITL_RECORD_BAG=${SITL_RECORD_BAG}" | tee -a "${LOG_FILE}"
fi

set +e
timeout "${TIMEOUT_SEC}" python3 "${SUPER_DRONE_DIR}/scripts/nationals_px4_sitl_pointcloud_super_validator.py" \
    _target_alt:="${TARGET_ALT}" \
    _summary_path:="${SUMMARY_PATH}" \
    _goal_dx:="${SITL_SUPER_GOAL_DX:-0.0}" \
    _goal_dy:="${SITL_SUPER_GOAL_DY:-1.0}" \
    _goal_dz:="${SITL_SUPER_GOAL_DZ:-0.0}" \
    _goal_tolerance:="${SITL_SUPER_GOAL_TOLERANCE:-1.0}" 2>&1 | tee -a "${LOG_FILE}"
RESULT=${PIPESTATUS[0]}
set -e

echo "[pointcloud_super] final summary:" | tee -a "${LOG_FILE}"
grep -E "\[pointcloud_super\] SUMMARY|\[pointcloud_super\] summary_path=|FAIL:|ERROR|TIMEOUT" "${LOG_FILE}" | tail -n 140 | tee -a "${LOG_FILE}" || true
echo "[pointcloud_super] summary: ${SUMMARY_PATH}" | tee -a "${LOG_FILE}"
echo "[pointcloud_super] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[pointcloud_super] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if [ "${RESULT}" -eq 0 ]; then
    hold_gazebo_gui_after_pass_if_requested
    echo "PASS: pointcloud_super_demo mission succeeded" | tee -a "${LOG_FILE}"
    exit 0
fi

echo "FAIL: pointcloud_super_demo mission failed with code ${RESULT}" | tee -a "${LOG_FILE}"
echo "See log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
exit "${RESULT}"

#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${WORKSPACE:-${HOME}/super_ws}}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${REPO:-${SUPER_WS}/src/SUPER_DRONE}}"
GEZOGO_DIR="${GEZOGO_DIR:-${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}}"
SEED="${SEED:-2026}"
HEADLESS="${HEADLESS:-1}"
START_RVIZ="${START_RVIZ:-1}"
SITL_RECORD_BAG="${SITL_RECORD_BAG:-0}"
MISSION_MAX_RINGS="${MISSION_MAX_RINGS:-1}"
TIMEOUT_SEC="${TIMEOUT_SEC:-360}"
ROS_PORT="${ROS_PORT:-11325}"
GAZEBO_PORT="${GAZEBO_PORT:-11347}"
TARGET_ALT="${SITL_FULL_CHAIN_ALT:-1.0}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${LAYOUT_PATH:-${GENERATED_DIR}/layout.json}"
LOG_DIR="${SUPER_DRONE_DIR}/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
WAYPOINTS="${WAYPOINTS:-${LOG_DIR}/nationals_full_chain_${STAMP}.waypoints.txt}"
LOG_FILE="${LOG_DIR}/nationals_full_chain_rviz_mission_${STAMP}.log"
BAG_PATH="${LOG_DIR}/nationals_full_chain_rviz_mission_${STAMP}.bag"
SUMMARY_PATH="${LOG_DIR}/nationals_full_chain_rviz_mission_${STAMP}.summary.json"
RVIZ_CONFIG="${RVIZ_CONFIG:-${SUPER_DRONE_DIR}/mission_planner/rviz/nationals_full_chain.rviz}"

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
RVIZ_STARTED=0
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
    echo "[full_chain] starting ${name}" | tee -a "${LOG_FILE}"
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
            echo "[full_chain] topic ready: ${topic}" | tee -a "${LOG_FILE}"
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
            echo "[full_chain] ready: ${label}" | tee -a "${LOG_FILE}"
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

wait_static_odom_for_takeoff() {
    local timeout_s="${SITL_FULL_CHAIN_STATIC_ODOM_WAIT_SEC:-30}"
    local vel_max="${SITL_FULL_CHAIN_STATIC_ODOM_VEL_MAX:-0.095}"
    echo "[full_chain] waiting for static odom before TAKEOFF vel<=${vel_max}m/s timeout=${timeout_s}s" | tee -a "${LOG_FILE}"
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
rospy.init_node("nationals_full_chain_static_odom_wait", anonymous=True, disable_signals=True)
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
                print(f"[full_chain] static odom ready vel={last_norm:.4f}m/s")
                sys.exit(0)
        else:
            stable_since = None
    rate.sleep()
print(f"FAIL: static odom timeout last_vel={last_norm:.4f}m/s threshold={vel_max:.4f}m/s")
sys.exit(1)
PY
}

start_rviz_if_requested() {
    if [ "${START_RVIZ}" != "1" ]; then
        echo "[rviz] START_RVIZ=${START_RVIZ}; not starting RViz" | tee -a "${LOG_FILE}"
        return 0
    fi
    if [ ! -f "${RVIZ_CONFIG}" ]; then
        echo "FAIL: RViz config not found: ${RVIZ_CONFIG}" | tee -a "${LOG_FILE}"
        return 1
    fi
    echo "[rviz] starting RViz with ROS_MASTER_URI=${ROS_MASTER_URI}" | tee -a "${LOG_FILE}"
    setsid env ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" rviz -d "${RVIZ_CONFIG}" >> "${LOG_FILE}" 2>&1 &
    local rviz_pid="$!"
    PIDS+=("${rviz_pid}")
    sleep 3
    if kill -0 "${rviz_pid}" >/dev/null 2>&1; then
        RVIZ_STARTED=1
        echo "[rviz] rviz_pid=${rviz_pid}" | tee -a "${LOG_FILE}"
    else
        RVIZ_STARTED=0
        echo "[rviz] RViz exited early; continuing headless validation" | tee -a "${LOG_FILE}"
    fi
}

if [ ! -f "${LAYOUT_PATH}" ]; then
    GENERATOR="${GEZOGO_DIR}/tools/generate_nationals_world.py"
    if [ ! -f "${GENERATOR}" ]; then
        echo "FAIL: layout missing and generator not found: ${GENERATOR}" >&2
        exit 1
    fi
    python3 "${GENERATOR}" --seed "${SEED}" 2>&1 | tee -a "${LOG_FILE}"
fi

python3 "${SUPER_DRONE_DIR}/mission_planner/scripts/generate_nationals_waypoints.py" \
    --layout "${LAYOUT_PATH}" \
    --output "${WAYPOINTS}" \
    --z "${TARGET_ALT}" \
    --landing-z "${TARGET_ALT}" \
    --switch-dis "${SITL_FULL_CHAIN_SWITCH_RADIUS:-1.00}" \
    --final-switch-dis "${SITL_FULL_CHAIN_FINAL_SWITCH_RADIUS:-0.80}" \
    --field-margin "${SITL_MISSION_FIELD_MARGIN:-1.50}" 2>&1 | tee -a "${LOG_FILE}"

echo "[full_chain] Nationals full-chain RViz mission" | tee -a "${LOG_FILE}"
echo "[full_chain] chain: layout_generated /cloud_registered -> SUPER /fsm_node -> /position_cmd -> px4ctrl -> /mavros/setpoint_raw/attitude -> MAVROS -> MAVLink UDP -> PX4 SITL -> Gazebo headless iris" | tee -a "${LOG_FILE}"
echo "[full_chain] summary: ${SUMMARY_PATH}" | tee -a "${LOG_FILE}"
echo "[full_chain] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[full_chain] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if ! rostopic list >/dev/null 2>&1; then
    run_bg roscore roscore -p "${ROS_PORT}"
    sleep 3
fi

run_bg px4_sitl env HEADLESS=1 SUPER_WS="${SUPER_WS}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4_sitl_world.sh"
sleep 10

run_bg mavros env SUPER_WS="${SUPER_WS}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_mavros.sh"
wait_topic /mavros/state 45
wait_grep "MAVROS connected=True" 45 bash -lc "rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'"

run_bg odom_relay env ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" rosrun topic_tools relay /mavros/local_position/odom /Odom_high_freq __name:=nationals_full_chain_odom_relay
wait_topic /Odom_high_freq 20

run_bg super env SUPER_WS="${SUPER_WS}" SUPER_DRONE_DIR="${SUPER_DRONE_DIR}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" LAYOUT_PATH="${LAYOUT_PATH}" WAYPOINTS="${WAYPOINTS}" SITL_MISSION_Z="${TARGET_ALT}" SITL_MISSION_LANDING_Z="${TARGET_ALT}" SITL_MISSION_SWITCH_DIS="${SITL_FULL_CHAIN_SWITCH_RADIUS:-1.00}" SITL_MISSION_FINAL_SWITCH_DIS="${SITL_FULL_CHAIN_FINAL_SWITCH_RADIUS:-0.80}" SITL_MISSION_FIELD_MARGIN="${SITL_MISSION_FIELD_MARGIN:-1.50}" SITL_START_MISSION=false SITL_START_SMOKE_GOAL=false SITL_PLANNER_CONFIG_NAME="${SITL_PLANNER_CONFIG_NAME:-super_drone_px4_sitl_smoke.yaml}" SITL_INCLUDE_BOUNDARY_WALLS="${SITL_INCLUDE_BOUNDARY_WALLS:-true}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_super_sitl_smoke.sh"
wait_topic /cloud_registered 45
wait_grep "fsm_node running" 45 bash -lc "rosnode list 2>/dev/null | grep -q '^/fsm_node$'"

start_rviz_if_requested

run_bg px4ctrl env SUPER_WS="${SUPER_WS}" SITL_TAKEOFF_HEIGHT="${TARGET_ALT}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4ctrl_sitl.sh"
wait_topic /mavros/setpoint_raw/attitude 45
wait_grep "px4ctrl subscribes /position_cmd" 45 bash -lc "rostopic info /position_cmd 2>/dev/null | grep -q '/px4ctrl'"
wait_static_odom_for_takeoff

if [ "${SITL_RECORD_BAG}" = "1" ]; then
    rosbag record -O "${BAG_PATH}" --lz4 \
        /cloud_registered \
        /Odom_high_freq \
        /mavros/local_position/odom \
        /planning/click_goal \
        /position_cmd \
        /mavros/state \
        /mavros/setpoint_raw/attitude \
        /px4ctrl/takeoff_land \
        /nationals_mission/waypoints \
        /nationals_mission/status \
        /nationals_mission/position_cmd_path \
        /nationals_mission/executed_path \
        /rosout_agg >> "${LOG_FILE}" 2>&1 &
    PIDS+=("$!")
    sleep 1
else
    echo "[full_chain] bag recording disabled by SITL_RECORD_BAG=${SITL_RECORD_BAG}" | tee -a "${LOG_FILE}"
fi

set +e
timeout "${TIMEOUT_SEC}" python3 "${SUPER_DRONE_DIR}/scripts/nationals_full_chain_mission_runner.py" \
    _target_alt:="${TARGET_ALT}" \
    _waypoints_path:="${WAYPOINTS}" \
    _summary_path:="${SUMMARY_PATH}" \
    _mission_max_rings:="${MISSION_MAX_RINGS}" \
    _rviz_started:="${RVIZ_STARTED}" \
    _rviz_config:="${RVIZ_CONFIG}" \
    _bag_recorded:="$([ "${SITL_RECORD_BAG}" = "1" ] && echo true || echo false)" \
    _bag_path:="$([ "${SITL_RECORD_BAG}" = "1" ] && echo "${BAG_PATH}" || true)" 2>&1 | tee -a "${LOG_FILE}"
RESULT=${PIPESTATUS[0]}
set -e

echo "[full_chain] final summary:" | tee -a "${LOG_FILE}"
grep -E "\[full_chain\] SUMMARY|\[full_chain\] summary_path=|FAIL:|ERROR|TIMEOUT" "${LOG_FILE}" | tail -n 140 | tee -a "${LOG_FILE}" || true
echo "[full_chain] summary: ${SUMMARY_PATH}" | tee -a "${LOG_FILE}"
echo "[full_chain] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[full_chain] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if [ "${RESULT}" -eq 0 ]; then
    echo "PASS: full_chain mission completed requested stage ${MISSION_MAX_RINGS}" | tee -a "${LOG_FILE}"
    exit 0
fi

echo "FAIL: full_chain mission failed with code ${RESULT}" | tee -a "${LOG_FILE}"
echo "See log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
exit "${RESULT}"

#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${WORKSPACE:-${HOME}/super_ws}}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${REPO:-${SUPER_WS}/src/SUPER_DRONE}}"
GEZOGO_DIR="${GEZOGO_DIR:-${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}}"
SEED="${SEED:-2026}"
HEADLESS="${HEADLESS:-1}"
TIMEOUT_SEC="${TIMEOUT_SEC:-300}"
ROS_PORT="${ROS_PORT:-11325}"
GAZEBO_PORT="${GAZEBO_PORT:-11347}"
TARGET_ALT="${SITL_DEMO_ALT:-1.30}"
SWITCH_RADIUS="${SITL_DEMO_SWITCH_RADIUS:-0.75}"
RING_SWITCH_RADIUS="${SITL_DEMO_RING_SWITCH_RADIUS:-1.00}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${LAYOUT_PATH:-${GENERATED_DIR}/layout.json}"
LOG_DIR="${SUPER_DRONE_DIR}/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
WAYPOINTS="${WAYPOINTS:-${LOG_DIR}/nationals_px4_sitl_position_cmd_demo_${STAMP}.waypoints.txt}"
LOG_FILE="${LOG_DIR}/nationals_px4_sitl_position_cmd_demo_${STAMP}.log"
BAG_PATH="${LOG_DIR}/nationals_px4_sitl_position_cmd_demo_${STAMP}.bag"
SUMMARY_PATH="${LOG_DIR}/nationals_px4_sitl_position_cmd_demo_${STAMP}.summary.json"

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

# shellcheck disable=SC1091
set +u
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${SUPER_WS}/devel/setup.bash"
set -u

for package in mavros px4ctrl quadrotor_msgs; do
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
    echo "[position_cmd_demo] starting ${name}" | tee -a "${LOG_FILE}"
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
            echo "[position_cmd_demo] topic ready: ${topic}" | tee -a "${LOG_FILE}"
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
            echo "[position_cmd_demo] ready: ${label}" | tee -a "${LOG_FILE}"
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
    --switch-dis "${SWITCH_RADIUS}" \
    --final-switch-dis "${SWITCH_RADIUS}" \
    --field-margin "${SITL_MISSION_FIELD_MARGIN:-1.50}" 2>&1 | tee -a "${LOG_FILE}"

echo "[position_cmd_demo] DEMO ONLY: direct /position_cmd driver. strict_super_planning=False" | tee -a "${LOG_FILE}"
echo "[position_cmd_demo] SITL only. Do not use on real hardware." | tee -a "${LOG_FILE}"
echo "[position_cmd_demo] summary: ${SUMMARY_PATH}" | tee -a "${LOG_FILE}"
echo "[position_cmd_demo] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[position_cmd_demo] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if ! rostopic list >/dev/null 2>&1; then
    run_bg roscore roscore -p "${ROS_PORT}"
    sleep 3
fi

run_bg px4_sitl env HEADLESS="${HEADLESS}" SUPER_WS="${SUPER_WS}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4_sitl_world.sh"
sleep 10

run_bg mavros env SUPER_WS="${SUPER_WS}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_mavros.sh"
wait_topic /mavros/state 45
wait_grep "MAVROS connected=True" 45 bash -lc "rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'"

run_bg odom_relay env ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" rosrun topic_tools relay /mavros/local_position/odom /Odom_high_freq __name:=nationals_position_cmd_demo_odom_relay
wait_topic /Odom_high_freq 20

run_bg px4ctrl env SUPER_WS="${SUPER_WS}" SITL_TAKEOFF_HEIGHT="${TARGET_ALT}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4ctrl_sitl.sh"
wait_grep "px4ctrl subscribes /position_cmd" 45 bash -lc "rostopic info /position_cmd 2>/dev/null | grep -q '/px4ctrl'"
wait_topic /mavros/setpoint_raw/attitude 45

if [ "${SITL_RECORD_BAG:-1}" = "1" ]; then
    rosbag record -O "${BAG_PATH}" --lz4 \
        /mavros/state \
        /mavros/local_position/odom \
        /Odom_high_freq \
        /position_cmd \
        /mavros/setpoint_raw/attitude \
        /px4ctrl/takeoff_land \
        /rosout_agg >> "${LOG_FILE}" 2>&1 &
    PIDS+=("$!")
    sleep 1
else
    echo "[position_cmd_demo] bag recording disabled by SITL_RECORD_BAG=0" | tee -a "${LOG_FILE}"
fi

set +e
timeout "${TIMEOUT_SEC}" python3 "${SUPER_DRONE_DIR}/scripts/nationals_px4_sitl_position_cmd_demo.py" \
    _target_alt:="${TARGET_ALT}" \
    _waypoints_path:="${WAYPOINTS}" \
    _summary_path:="${SUMMARY_PATH}" \
    _switch_radius:="${SWITCH_RADIUS}" \
    _ring_switch_radius:="${RING_SWITCH_RADIUS}" 2>&1 | tee -a "${LOG_FILE}"
RESULT=${PIPESTATUS[0]}
set -e

echo "[position_cmd_demo] final summary:" | tee -a "${LOG_FILE}"
grep -E "\[position_cmd_demo\] SUMMARY|\[position_cmd_demo\] summary_path=|FAIL:|ERROR|abort|TIMEOUT" "${LOG_FILE}" | tail -n 120 | tee -a "${LOG_FILE}" || true

echo "[position_cmd_demo] summary: ${SUMMARY_PATH}" | tee -a "${LOG_FILE}"
echo "[position_cmd_demo] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[position_cmd_demo] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if [ "${RESULT}" -eq 0 ]; then
    echo "PASS: position_cmd_demo mission succeeded" | tee -a "${LOG_FILE}"
    exit 0
fi

echo "FAIL: position_cmd_demo mission failed with code ${RESULT}" | tee -a "${LOG_FILE}"
echo "See log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
exit "${RESULT}"

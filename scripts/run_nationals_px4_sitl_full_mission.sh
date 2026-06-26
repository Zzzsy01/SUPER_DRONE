#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${WORKSPACE:-${HOME}/super_ws}}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${REPO:-${SUPER_WS}/src/SUPER_DRONE}}"
GEZOGO_DIR="${GEZOGO_DIR:-${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}}"
SEED="${SEED:-2026}"
STAGE="${STAGE:-first_ring}"
HEADLESS="${HEADLESS:-0}"
TIMEOUT_SEC="${TIMEOUT_SEC:-240}"
DEMO_ALT="${SITL_DEMO_ALT:-1.30}"
DEMO_SPACING="${SITL_DEMO_SPACING:-0.60}"
DEMO_SWITCH_RADIUS="${SITL_DEMO_SWITCH_RADIUS:-1.00}"
ROS_PORT="${ROS_PORT:-11325}"
GAZEBO_PORT="${GAZEBO_PORT:-11347}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${LAYOUT_PATH:-${GENERATED_DIR}/layout.json}"
WAYPOINTS="${WAYPOINTS:-${SUPER_DRONE_DIR}/mission_planner/data/nationals_seed_${SEED}.txt}"
LOG_DIR="${SUPER_DRONE_DIR}/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/nationals_px4_sitl_${STAGE}_${STAMP}.log"
BAG_PATH="${LOG_DIR}/nationals_px4_sitl_${STAGE}_${STAMP}.bag"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/super_drone_roslog}"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
mkdir -p "${LOG_DIR}" "${ROS_LOG_DIR}" "${ROS_HOME}"

if [ "${STAGE}" != "first_ring" ] && [ "${STAGE}" != "stack" ] && [ "${STAGE}" != "demo_full_route" ]; then
    echo "FAIL: this SITL script supports STAGE=stack, STAGE=first_ring, or STAGE=demo_full_route only; refusing strict full 5/5 run." >&2
    exit 2
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "FAIL: /opt/ros/noetic/setup.bash not found" >&2
    exit 1
fi
if [ ! -f "${SUPER_WS}/devel/setup.bash" ]; then
    echo "FAIL: ${SUPER_WS}/devel/setup.bash not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first." >&2
    exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${SUPER_WS}/devel/setup.bash"
set -u

for package in mavros px4ctrl mission_planner super_planner quadrotor_msgs; do
    if ! rospack find "${package}" >/dev/null 2>&1; then
        echo "FAIL: ROS package ${package} is not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first." >&2
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
    echo "[${STAGE}] starting ${name}" | tee -a "${LOG_FILE}"
    if [ "${STAGE}" = "demo_full_route" ] && [ "${name}" = "px4_sitl" ]; then
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
            echo "[${STAGE}] topic ready: ${topic}" | tee -a "${LOG_FILE}"
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

if [ ! -f "${LAYOUT_PATH}" ]; then
    GENERATOR="${GEZOGO_DIR}/tools/generate_nationals_world.py"
    if [ ! -f "${GENERATOR}" ]; then
        echo "FAIL: layout missing and generator not found: ${GENERATOR}" >&2
        exit 1
    fi
    python3 "${GENERATOR}" --seed "${SEED}" 2>&1 | tee -a "${LOG_FILE}"
fi

MISSION_Z="${SITL_MISSION_Z:-1.10}"
MISSION_LANDING_Z="${SITL_MISSION_LANDING_Z:-1.00}"
if [ "${STAGE}" = "demo_full_route" ]; then
    MISSION_Z="${DEMO_ALT}"
    MISSION_LANDING_Z="${DEMO_ALT}"
fi

python3 "${SUPER_DRONE_DIR}/mission_planner/scripts/generate_nationals_waypoints.py" \
    --layout "${LAYOUT_PATH}" \
    --output "${WAYPOINTS}" \
    --z "${MISSION_Z}" \
    --landing-z "${MISSION_LANDING_Z}" \
    --switch-dis "${SITL_MISSION_SWITCH_DIS:-0.90}" \
    --final-switch-dis "${SITL_MISSION_FINAL_SWITCH_DIS:-0.60}" \
    --field-margin "${SITL_MISSION_FIELD_MARGIN:-1.50}" 2>&1 | tee -a "${LOG_FILE}"

echo "[${STAGE}] SITL only. Do not use on real hardware." | tee -a "${LOG_FILE}"
if [ "${STAGE}" = "demo_full_route" ]; then
    echo "[${STAGE}] DEMO ONLY: Gazebo/PX4 SITL display/recording route; not strict competition validation." | tee -a "${LOG_FILE}"
fi
echo "[${STAGE}] log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "[${STAGE}] bag: ${BAG_PATH}" | tee -a "${LOG_FILE}"

if ! rostopic list >/dev/null 2>&1; then
    run_bg roscore roscore -p "${ROS_PORT}"
    sleep 3
fi

run_bg px4_sitl env HEADLESS="${HEADLESS}" SUPER_WS="${SUPER_WS}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4_sitl_world.sh"
wait_topic /mavros/state 1 >/dev/null 2>&1 || true
sleep 10

run_bg mavros env SUPER_WS="${SUPER_WS}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_mavros.sh"
wait_topic /mavros/state 45

PX4CTRL_TAKEOFF_HEIGHT="${SITL_TAKEOFF_HEIGHT:-1.0}"
if [ "${STAGE}" = "demo_full_route" ]; then
    PX4CTRL_TAKEOFF_HEIGHT="${DEMO_ALT}"
fi
run_bg px4ctrl env SUPER_WS="${SUPER_WS}" SITL_TAKEOFF_HEIGHT="${PX4CTRL_TAKEOFF_HEIGHT}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_px4ctrl_sitl.sh"
wait_topic /mavros/setpoint_raw/attitude 45

PLANNER_CONFIG="${SITL_PLANNER_CONFIG_NAME:-super_drone_px4_sitl_first_ring.yaml}"
INCLUDE_BOUNDARY_WALLS="${SITL_INCLUDE_BOUNDARY_WALLS:-true}"
if [ "${STAGE}" = "demo_full_route" ]; then
    PLANNER_CONFIG="${SITL_PLANNER_CONFIG_NAME:-super_drone_px4_sitl_demo.yaml}"
    INCLUDE_BOUNDARY_WALLS="${SITL_DEMO_INCLUDE_BOUNDARY_WALLS:-false}"
fi
run_bg super env SUPER_WS="${SUPER_WS}" SUPER_DRONE_DIR="${SUPER_DRONE_DIR}" GEZOGO_DIR="${GEZOGO_DIR}" SEED="${SEED}" LAYOUT_PATH="${LAYOUT_PATH}" WAYPOINTS="${WAYPOINTS}" SITL_START_MISSION=false SITL_START_SMOKE_GOAL=false SITL_PLANNER_CONFIG_NAME="${PLANNER_CONFIG}" SITL_INCLUDE_BOUNDARY_WALLS="${INCLUDE_BOUNDARY_WALLS}" ROS_MASTER_URI="${ROS_MASTER_URI}" ROS_IP="${ROS_IP}" ROS_HOSTNAME="${ROS_HOSTNAME}" ROS_LOG_DIR="${ROS_LOG_DIR}" ROS_HOME="${ROS_HOME}" "${SUPER_DRONE_DIR}/scripts/run_nationals_super_sitl_smoke.sh"
wait_topic /cloud_registered 45
wait_topic /Odom_high_freq 45

if [ "${SITL_RECORD_BAG:-1}" = "1" ]; then
    BAG_TOPICS=(
        /mavros/state
        /mavros/local_position/odom
        /Odom_high_freq
        /planning/click_goal
        /position_cmd
        /mavros/setpoint_raw/attitude
        /rosout_agg
    )
    if [ "${STAGE}" != "demo_full_route" ]; then
        BAG_TOPICS+=(/cloud_registered)
    fi
    rosbag record -O "${BAG_PATH}" --lz4 "${BAG_TOPICS[@]}" >> "${LOG_FILE}" 2>&1 &
    PIDS+=("$!")
    sleep 1
else
    echo "[${STAGE}] bag recording disabled by SITL_RECORD_BAG=0" | tee -a "${LOG_FILE}"
fi

if [ "${STAGE}" = "stack" ]; then
    set +e
    timeout "${TIMEOUT_SEC}" python3 "${SUPER_DRONE_DIR}/scripts/nationals_px4_sitl_frame_check.py" \
        _layout_path:="${LAYOUT_PATH}" \
        _target_alt:="${SITL_FIRST_RING_TARGET_ALT:-1.0}" \
        _field_margin:="${SITL_FIRST_RING_FIELD_MARGIN:-0.0}" \
        _frame_scale:="${SITL_FRAME_SCALE:--1,1,1}" \
        _frame_offset:="${SITL_FRAME_OFFSET:-3,-1,0}" 2>&1 | tee -a "${LOG_FILE}"
    RESULT=${PIPESTATUS[0]}
    set -e
    echo "[stack] final summary:" | tee -a "${LOG_FILE}"
    grep -E "\[frame_check\]|Local start point is deeply occupied|PathSearch for new path failed|GenerateExpTrajectory failed|Odom below virtual ground" "${LOG_FILE}" | tail -n 80 | tee -a "${LOG_FILE}" || true
    if [ "${RESULT}" -eq 0 ]; then
        echo "PASS: stack frame check succeeded" | tee -a "${LOG_FILE}"
        exit 0
    fi
    echo "FAIL: stack frame check failed with code ${RESULT}" | tee -a "${LOG_FILE}"
    echo "See log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
    exit "${RESULT}"
fi

set +e
DRIVER_TARGET_ALT="${SITL_FIRST_RING_TARGET_ALT:-1.0}"
if [ "${STAGE}" = "demo_full_route" ]; then
    DRIVER_TARGET_ALT="${DEMO_ALT}"
fi
timeout "${TIMEOUT_SEC}" python3 "${SUPER_DRONE_DIR}/scripts/nationals_px4_sitl_first_ring.py" \
    _layout_path:="${LAYOUT_PATH}" \
    _waypoints_path:="${WAYPOINTS}" \
    _mission_mode:="${STAGE}" \
    _target_alt:="${DRIVER_TARGET_ALT}" \
    _mid_switch_radius:="${SITL_FIRST_RING_MID_SWITCH_RADIUS:-0.90}" \
    _ring_switch_radius:="${SITL_FIRST_RING_RING_SWITCH_RADIUS:-1.10}" \
    _demo_spacing:="${DEMO_SPACING}" \
    _demo_switch_radius:="${DEMO_SWITCH_RADIUS}" \
    _demo_min_reached:="${SITL_DEMO_MIN_REACHED:-4}" \
    _goal_publish_rate:="${SITL_FIRST_RING_GOAL_PUBLISH_RATE:-0.5}" \
    _field_margin:="${SITL_FIRST_RING_FIELD_MARGIN:-0.35}" \
    _frame_scale:="${SITL_FRAME_SCALE:--1,1,1}" \
    _frame_offset:="${SITL_FRAME_OFFSET:-3,-1,0}" 2>&1 | tee -a "${LOG_FILE}"
RESULT=${PIPESTATUS[0]}
set -e

echo "[${STAGE}] final summary:" | tee -a "${LOG_FILE}"
grep -E "\[(first_ring|demo_full_route)\] SUMMARY|GeneratePolytopeFromLine failed|generateBackupTrajectory return FAILED|Cannot generate feasible backup sfc|GenerateExpTrajectory failed|Local start point is deeply occupied|PathSearch for new path failed|Odom below virtual ground" "${LOG_FILE}" | tail -n 120 | tee -a "${LOG_FILE}" || true

if [ "${RESULT}" -eq 0 ]; then
    echo "PASS: ${STAGE} mission succeeded" | tee -a "${LOG_FILE}"
    exit 0
fi

echo "FAIL: ${STAGE} mission failed with code ${RESULT}" | tee -a "${LOG_FILE}"
echo "See log: ${LOG_FILE}" | tee -a "${LOG_FILE}"
exit "${RESULT}"

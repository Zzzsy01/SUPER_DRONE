#!/usr/bin/env bash
set -u
set -o pipefail

WORKSPACE="${HOME}/super_ws"
REPO="${WORKSPACE}/src/SUPER_DRONE"
GUOSAI_REPO="${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}"
SEED="${SEED:-2026}"
TIMEOUT_SEC="${TIMEOUT_SEC:-180}"
ROS_PORT="${ROS_PORT:-11324}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/super_drone_roslog}"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO}/logs"
LOG_FILE="${LOG_DIR}/nationals_super_${STAMP}.log"
WAYPOINTS="${REPO}/mission_planner/data/nationals_seed_${SEED}.txt"

mkdir -p "${LOG_DIR}" "${ROS_LOG_DIR}" "${ROS_HOME}"
echo "[nationals_super] log: ${LOG_FILE}"

find_layout() {
    for candidate in \
        "${GUOSAI_REPO}/layout.json" \
        "${GUOSAI_REPO}/worlds/layout.json" \
        "${GUOSAI_REPO}/generated/layout.json" \
        "${GUOSAI_REPO}/output/layout.json" \
        "${GUOSAI_REPO}/nationals_seed_${SEED}/layout.json"; do
        if [ -f "${candidate}" ]; then
            echo "${candidate}"
            return 0
        fi
    done
    find "${GUOSAI_REPO}" -maxdepth 4 -name layout.json -print -quit 2>/dev/null
}

LAYOUT_PATH="$(find_layout)"
if [ -z "${LAYOUT_PATH}" ]; then
    GENERATOR="${GUOSAI_REPO}/tools/generate_nationals_world.py"
    if [ ! -f "${GENERATOR}" ]; then
        echo "FAIL: layout.json not found and generator is missing: ${GENERATOR}"
        exit 1
    fi
    echo "[nationals_super] generating layout with seed ${SEED}" | tee -a "${LOG_FILE}"
    if ! python3 "${GENERATOR}" --seed "${SEED}" 2>&1 | tee -a "${LOG_FILE}"; then
        echo "FAIL: generate_nationals_world.py failed"
        exit 1
    fi
    LAYOUT_PATH="$(find_layout)"
fi

if [ -z "${LAYOUT_PATH}" ] || [ ! -f "${LAYOUT_PATH}" ]; then
    echo "FAIL: layout.json not found after generation"
    exit 1
fi
echo "[nationals_super] layout: ${LAYOUT_PATH}" | tee -a "${LOG_FILE}"

cd "${WORKSPACE}" || exit 1
if ! catkin_make -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "${LOG_FILE}"; then
    echo "FAIL: catkin_make failed"
    exit 1
fi

# shellcheck disable=SC1091
source "${WORKSPACE}/devel/setup.bash"

if ! python3 "${REPO}/mission_planner/scripts/generate_nationals_waypoints.py" \
    --layout "${LAYOUT_PATH}" \
    --output "${WAYPOINTS}" \
    --z "${VALIDATION_Z:-1.10}" \
    --landing-z "${VALIDATION_LANDING_Z:-1.00}" \
    --switch-dis "${VALIDATION_SWITCH_DIS:-0.90}" \
    --final-switch-dis "${VALIDATION_FINAL_SWITCH_DIS:-0.60}" \
    --field-margin "${VALIDATION_FIELD_MARGIN:-1.50}" 2>&1 | tee -a "${LOG_FILE}"; then
    echo "FAIL: generate_nationals_waypoints.py failed"
    exit 1
fi

setsid bash -c "source '${WORKSPACE}/devel/setup.bash' && export ROS_MASTER_URI='${ROS_MASTER_URI}' ROS_IP='${ROS_IP}' ROS_HOSTNAME='${ROS_HOSTNAME}' ROS_LOG_DIR='${ROS_LOG_DIR}' ROS_HOME='${ROS_HOME}' GAZEBO_MASTER_URI='${GAZEBO_MASTER_URI}' && roslaunch --local mission_planner nationals_super_mock.launch layout_path:='${LAYOUT_PATH}' waypoints_path:='${WAYPOINTS}'" \
    >> "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

START_TIME="$(date +%s)"
RESULT=""
while true; do
    if grep -q "\[PASS\] nationals super validation" "${LOG_FILE}"; then
        RESULT="PASS"
        break
    fi
    if grep -q "\[FAIL\] nationals super validation" "${LOG_FILE}"; then
        RESULT="FAIL"
        break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        RESULT="FAIL: roslaunch exited before validator result"
        break
    fi
    NOW="$(date +%s)"
    if [ "$((NOW - START_TIME))" -ge "${TIMEOUT_SEC}" ]; then
        RESULT="FAIL: validation timed out after ${TIMEOUT_SEC}s"
        break
    fi
    sleep 1
done

kill -TERM "-${LAUNCH_PID}" 2>/dev/null || true
sleep 2
kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true

echo "[nationals_super] final validator output:"
grep -A 10 -E "\[PASS\] nationals super validation|\[FAIL\] nationals super validation" "${LOG_FILE}" | tail -n 24 || true

if [ "${RESULT}" = "PASS" ]; then
    echo "PASS: nationals super validation succeeded"
    exit 0
fi

echo "FAIL: ${RESULT}"
echo "See log: ${LOG_FILE}"
exit 1

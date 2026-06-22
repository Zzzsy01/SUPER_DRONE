#!/usr/bin/env bash
set -u
set -o pipefail

WORKSPACE="${HOME}/super_ws"
REPO="${WORKSPACE}/src/SUPER_DRONE"
TIMEOUT_SEC="${TIMEOUT_SEC:-180}"
ROS_PORT="${ROS_PORT:-11322}"
export ROS_MASTER_URI="http://localhost:${ROS_PORT}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO}/logs"
LOG_FILE="${LOG_DIR}/gazebo_planning_${STAMP}.log"

mkdir -p "${LOG_DIR}"
echo "[gazebo_planning] log: ${LOG_FILE}"

cd "${WORKSPACE}" || exit 1
if ! catkin_make -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "${LOG_FILE}"; then
    echo "FAIL: catkin_make failed"
    exit 1
fi

# shellcheck disable=SC1091
source "${WORKSPACE}/devel/setup.bash"

setsid bash -c "source '${WORKSPACE}/devel/setup.bash' && export ROS_MASTER_URI='${ROS_MASTER_URI}' ROS_HOSTNAME='${ROS_HOSTNAME}' && roslaunch mission_planner gazebo_planning_sim.launch gui:=false rviz:=false" \
    >> "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

START_TIME="$(date +%s)"
RESULT=""
while true; do
    if grep -q "\[PASS\]" "${LOG_FILE}"; then
        RESULT="PASS"
        break
    fi
    if grep -q "\[FAIL\]" "${LOG_FILE}"; then
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
pkill -TERM -f "gzserver.*competition_smoke.world" 2>/dev/null || true
pkill -TERM -f "gzclient" 2>/dev/null || true
sleep 2
kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
pkill -KILL -f "gzserver.*competition_smoke.world" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true

echo "[gazebo_planning] final validator output:"
grep -A 9 -E "\[PASS\]|\[FAIL\]" "${LOG_FILE}" | tail -n 20 || true

if [ "${RESULT}" = "PASS" ]; then
    echo "PASS: gazebo planning validation succeeded"
    exit 0
fi

echo "FAIL: ${RESULT}"
echo "See log: ${LOG_FILE}"
exit 1

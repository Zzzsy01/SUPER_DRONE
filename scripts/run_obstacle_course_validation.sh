#!/usr/bin/env bash
set -u
set -o pipefail

WORKSPACE="${HOME}/super_ws"
REPO="${WORKSPACE}/src/SUPER_DRONE"
TIMEOUT_SEC="${TIMEOUT_SEC:-180}"
ROS_PORT="${ROS_PORT:-11323}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/super_drone_roslog}"
unset ROS_HOSTNAME
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO}/logs"
LOG_FILE="${LOG_DIR}/obstacle_course_${STAMP}.log"

mkdir -p "${LOG_DIR}" "${ROS_LOG_DIR}"
echo "[obstacle_course] log: ${LOG_FILE}"

cd "${WORKSPACE}" || exit 1
if ! catkin_make -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "${LOG_FILE}"; then
    echo "FAIL: catkin_make failed"
    exit 1
fi

# shellcheck disable=SC1091
source "${WORKSPACE}/devel/setup.bash"

setsid bash -c "source '${WORKSPACE}/devel/setup.bash' && unset ROS_HOSTNAME && export ROS_MASTER_URI='${ROS_MASTER_URI}' ROS_IP='${ROS_IP}' ROS_LOG_DIR='${ROS_LOG_DIR}' && roslaunch mission_planner gazebo_obstacle_course.launch gui:=false rviz:=false auto_goal:=true" \
    >> "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

START_TIME="$(date +%s)"
RESULT=""
while true; do
    if grep -q "\[PASS\] obstacle course validation" "${LOG_FILE}"; then
        RESULT="PASS"
        break
    fi
    if grep -q "\[FAIL\] obstacle course validation" "${LOG_FILE}"; then
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
pkill -TERM -f "gzserver.*obstacle_course.world" 2>/dev/null || true
pkill -TERM -f "gzclient" 2>/dev/null || true
sleep 2
kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
pkill -KILL -f "gzserver.*obstacle_course.world" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true

echo "[obstacle_course] final validator output:"
grep -A 10 -E "\[PASS\] obstacle course validation|\[FAIL\] obstacle course validation" "${LOG_FILE}" | tail -n 24 || true

if [ "${RESULT}" = "PASS" ]; then
    echo "PASS: obstacle course validation succeeded"
    exit 0
fi

echo "FAIL: ${RESULT}"
echo "See log: ${LOG_FILE}"
exit 1

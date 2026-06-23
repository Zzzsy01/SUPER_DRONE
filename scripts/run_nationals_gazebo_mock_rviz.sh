#!/usr/bin/env bash
set -u
set -o pipefail

WORKSPACE="${HOME}/super_ws"
REPO="${WORKSPACE}/src/SUPER_DRONE"
GUOSAI_REPO="${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}"
SEED="${SEED:-2026}"
ROS_PORT="${ROS_PORT:-11325}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/super_drone_roslog}"
unset ROS_HOSTNAME

GENERATOR="${GUOSAI_REPO}/tools/generate_nationals_world.py"
GENERATED_DIR="${GUOSAI_REPO}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${GENERATED_DIR}/layout.json"
WORLD_PATH="${GENERATED_DIR}/nationals_field.world"
WAYPOINTS="${TMPDIR:-/tmp}/super_drone_nationals_seed_${SEED}_visual.txt"

cleanup() {
    if [ -n "${LAUNCH_PID:-}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill -TERM "-${LAUNCH_PID}" 2>/dev/null || true
        sleep 2
        kill -KILL "-${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
    pkill -TERM -f "gzserver.*nationals_field.world" 2>/dev/null || true
    pkill -TERM -f "gzclient" 2>/dev/null || true
    pkill -TERM -f "rviz.*nationals_gazebo_mock.rviz" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mkdir -p "${ROS_LOG_DIR}"

if [ ! -f "${GENERATOR}" ]; then
    echo "FAIL: missing generator: ${GENERATOR}"
    exit 1
fi

echo "[nationals_gazebo_mock] generating world with seed ${SEED}"
python3 "${GENERATOR}" --seed "${SEED}" || exit 1

if [ ! -f "${LAYOUT_PATH}" ] || [ ! -f "${WORLD_PATH}" ]; then
    echo "FAIL: generated layout/world not found in ${GENERATED_DIR}"
    exit 1
fi

cd "${WORKSPACE}" || exit 1
catkin_make -DCMAKE_BUILD_TYPE=Release || exit 1

# shellcheck disable=SC1091
source "${WORKSPACE}/devel/setup.bash"

python3 "${REPO}/mission_planner/scripts/generate_nationals_waypoints.py" \
    --layout "${LAYOUT_PATH}" \
    --output "${WAYPOINTS}" \
    --z "${NATIONALS_MOCK_Z:-1.10}" \
    --landing-z "${NATIONALS_MOCK_LANDING_Z:-1.00}" \
    --switch-dis 0.90 \
    --final-switch-dis 0.60 \
    --field-margin 1.50 || exit 1

echo "[nationals_gazebo_mock] world: ${WORLD_PATH}"
echo "[nationals_gazebo_mock] layout: ${LAYOUT_PATH}"
echo "[nationals_gazebo_mock] waypoints: ${WAYPOINTS}"
echo "[nationals_gazebo_mock] starting Gazebo + SUPER mock + RViz. Press Ctrl+C to stop."

setsid bash -c "source '${WORKSPACE}/devel/setup.bash' && unset ROS_HOSTNAME && export ROS_MASTER_URI='${ROS_MASTER_URI}' ROS_IP='${ROS_IP}' ROS_LOG_DIR='${ROS_LOG_DIR}' && roslaunch mission_planner nationals_gazebo_mock_visual.launch world_path:='${WORLD_PATH}' layout_path:='${LAYOUT_PATH}' waypoints_path:='${WAYPOINTS}' gui:=true rviz:=true" &
LAUNCH_PID=$!
wait "${LAUNCH_PID}"

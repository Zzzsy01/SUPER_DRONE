#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${WORKSPACE:-${HOME}/super_ws}}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${REPO:-${SUPER_WS}/src/SUPER_DRONE}}"
GEZOGO_DIR="${GEZOGO_DIR:-${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}}"
SEED="${SEED:-2026}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${LAYOUT_PATH:-${GENERATED_DIR}/layout.json}"
WORLD_PATH="${GENERATED_DIR}/nationals_field.world"
GENERATOR="${GEZOGO_DIR}/tools/generate_nationals_world.py"
WAYPOINTS="${WAYPOINTS:-${SUPER_DRONE_DIR}/mission_planner/data/nationals_seed_${SEED}.txt}"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
mkdir -p "${ROS_HOME}"

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

for package in mission_planner super_planner quadrotor_msgs; do
    if ! rospack find "${package}" >/dev/null 2>&1; then
        echo "FAIL: ROS package ${package} is not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first." >&2
        exit 1
    fi
done

if [ ! -f "${WORLD_PATH}" ] || [ ! -f "${LAYOUT_PATH}" ]; then
    if [ ! -f "${GENERATOR}" ]; then
        echo "FAIL: nationals world/layout missing and generator is not found: ${GENERATOR}" >&2
        exit 1
    fi
    echo "[nationals_super_sitl] generating nationals world seed ${SEED}"
    python3 "${GENERATOR}" --seed "${SEED}"
fi

if [ ! -f "${LAYOUT_PATH}" ]; then
    echo "FAIL: layout not found: ${LAYOUT_PATH}" >&2
    exit 1
fi

python3 "${SUPER_DRONE_DIR}/mission_planner/scripts/generate_nationals_waypoints.py" \
    --layout "${LAYOUT_PATH}" \
    --output "${WAYPOINTS}" \
    --z "${SITL_MISSION_Z:-1.10}" \
    --landing-z "${SITL_MISSION_LANDING_Z:-1.00}" \
    --switch-dis "${SITL_MISSION_SWITCH_DIS:-0.90}" \
    --final-switch-dis "${SITL_MISSION_FINAL_SWITCH_DIS:-0.60}" \
    --field-margin "${SITL_MISSION_FIELD_MARGIN:-1.50}"

cd "${SUPER_WS}"

echo "[nationals_super_sitl] starting relay, layout cloud, SUPER fsm_node, and one safe /planning/click_goal smoke trigger"
echo "[nationals_super_sitl] SITL only: this script does not start px4ctrl, arm, takeoff, land, or target real hardware"
exec roslaunch mission_planner nationals_px4_sitl_smoke.launch \
    layout_path:="${LAYOUT_PATH}" \
    waypoints_path:="${WAYPOINTS}" \
    start_mission:="${SITL_START_MISSION:-false}" \
    start_smoke_goal:="${SITL_START_SMOKE_GOAL:-true}" \
    smoke_goal_x:="${SITL_SMOKE_GOAL_X:-3.000}" \
    smoke_goal_y:="${SITL_SMOKE_GOAL_Y:-1.000}" \
    smoke_goal_z:="${SITL_SMOKE_GOAL_Z:-1.000}" \
    smoke_goal_delay:="${SITL_SMOKE_GOAL_DELAY:-8.0}"

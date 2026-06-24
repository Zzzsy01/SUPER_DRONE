#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-${HOME}/super_ws}"
REPO="${REPO:-${WORKSPACE}/src/SUPER_DRONE}"
GUOSAI_REPO="${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}"
SEED="${SEED:-2026}"
GENERATED_DIR="${GUOSAI_REPO}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${LAYOUT_PATH:-${GENERATED_DIR}/layout.json}"
WORLD_PATH="${GENERATED_DIR}/nationals_field.world"
GENERATOR="${GUOSAI_REPO}/tools/generate_nationals_world.py"
WAYPOINTS="${WAYPOINTS:-${REPO}/mission_planner/data/nationals_seed_${SEED}.txt}"

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

python3 "${REPO}/mission_planner/scripts/generate_nationals_waypoints.py" \
    --layout "${LAYOUT_PATH}" \
    --output "${WAYPOINTS}" \
    --z "${SITL_MISSION_Z:-1.10}" \
    --landing-z "${SITL_MISSION_LANDING_Z:-1.00}" \
    --switch-dis "${SITL_MISSION_SWITCH_DIS:-0.90}" \
    --final-switch-dis "${SITL_MISSION_FINAL_SWITCH_DIS:-0.60}" \
    --field-margin "${SITL_MISSION_FIELD_MARGIN:-1.50}"

cd "${WORKSPACE}"
# shellcheck disable=SC1091
source "${WORKSPACE}/devel/setup.bash"

echo "[nationals_super_sitl] starting relay, layout cloud, SUPER fsm_node, and mission planner"
echo "[nationals_super_sitl] this script does not start px4ctrl, arm, takeoff, or land"
exec roslaunch mission_planner nationals_px4_sitl_smoke.launch \
    layout_path:="${LAYOUT_PATH}" \
    waypoints_path:="${WAYPOINTS}"

#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${HOME}/super_ws}"
GEZOGO_DIR="${GEZOGO_DIR:-${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}}"
SEED="${SEED:-2026}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
WORLD_PATH="${GENERATED_DIR}/nationals_field.world"
LAYOUT_PATH="${GENERATED_DIR}/layout.json"
GENERATOR="${GEZOGO_DIR}/tools/generate_nationals_world.py"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
mkdir -p "${ROS_HOME}"

source_ros() {
    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        echo "FAIL: /opt/ros/noetic/setup.bash not found" >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    set +u
    source /opt/ros/noetic/setup.bash
    if [ -f "${SUPER_WS}/devel/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "${SUPER_WS}/devel/setup.bash"
    else
        echo "FAIL: ${SUPER_WS}/devel/setup.bash not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first." >&2
        exit 1
    fi
    set -u
}

find_px4_dir() {
    if [ -n "${PX4_DIR:-}" ]; then
        [ -d "${PX4_DIR}" ] && readlink -f "${PX4_DIR}" && return 0
        return 1
    fi
    local candidate
    for candidate in \
        "${HOME}/PX4-Autopilot" \
        "${HOME}/ws/PX4-Autopilot" \
        "${HOME}/src/PX4-Autopilot" \
        "${HOME}/super_ws/src/PX4-Autopilot"; do
        [ -d "${candidate}" ] && readlink -f "${candidate}" && return 0
    done
    return 1
}

source_ros

PX4_FOUND_DIR="$(find_px4_dir || true)"
if [ -z "${PX4_FOUND_DIR}" ]; then
    echo "FAIL: PX4-Autopilot not found." >&2
    echo "      Search paths: ${HOME}/PX4-Autopilot, ${HOME}/ws/PX4-Autopilot, ${HOME}/src/PX4-Autopilot, ${HOME}/super_ws/src/PX4-Autopilot" >&2
    echo "      Clone/build PX4 or run with PX4_DIR=/path/to/PX4-Autopilot ./scripts/run_nationals_px4_sitl_world.sh" >&2
    exit 1
fi
PX4_DIR="${PX4_FOUND_DIR}"

if [ ! -f "${WORLD_PATH}" ] || [ ! -f "${LAYOUT_PATH}" ]; then
    if [ ! -f "${GENERATOR}" ]; then
        echo "FAIL: world/layout missing and generator is not found: ${GENERATOR}" >&2
        exit 1
    fi
    echo "[nationals_px4_sitl] generating nationals world seed ${SEED}"
    python3 "${GENERATOR}" --seed "${SEED}"
fi

if [ ! -f "${WORLD_PATH}" ]; then
    echo "FAIL: nationals world was not generated: ${WORLD_PATH}" >&2
    exit 1
fi

echo "[nationals_px4_sitl] PX4_SITL_WORLD=${WORLD_PATH}"
echo "[nationals_px4_sitl] starting PX4 SITL + Gazebo Classic iris"
echo "[nationals_px4_sitl] SITL only: this script does not start SUPER, px4ctrl, MAVROS, arm, takeoff, land, real hardware, or mission goals"

export PX4_SITL_WORLD="${WORLD_PATH}"
exec make -C "${PX4_DIR}" px4_sitl_default gazebo-classic_iris

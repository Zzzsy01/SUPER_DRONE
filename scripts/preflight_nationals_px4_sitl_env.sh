#!/usr/bin/env bash
set -u
set -o pipefail

SUPER_WS="${SUPER_WS:-${HOME}/super_ws}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${SUPER_WS}/src/SUPER_DRONE}"
GEZOGO_DIR="${GEZOGO_DIR:-${HOME}/ws/gezogo-guosai}"
SEED="${SEED:-2026}"
GENERATED_DIR="${GEZOGO_DIR}/gazebo_px4_nationals/generated/seed_${SEED}"
LAYOUT_PATH="${GENERATED_DIR}/layout.json"
WORLD_PATH="${GENERATED_DIR}/nationals_field.world"
GENERATOR="${GEZOGO_DIR}/tools/generate_nationals_world.py"
GEOID_PATH="/usr/share/GeographicLib/geoids/egm96-5.pgm"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
mkdir -p "${ROS_HOME}"

FAIL=0
PX4_STATUS="missing"
GEOGRAPHICLIB_STATUS="missing"
NATIONALS_SIM_STATUS="missing"
PX4CTRL_STATUS="missing"
CATKIN_STATUS="not run"

fail() {
    echo "FAIL: $*" >&2
    FAIL=1
}

ok() {
    echo "OK: $*"
}

source_ros() {
    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        fail "/opt/ros/noetic/setup.bash not found"
        return 1
    fi
    # shellcheck disable=SC1091
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
}

resolve_dir() {
    local path="$1"
    if [ -e "${path}" ]; then
        readlink -f "${path}"
    fi
}

link_package() {
    local package_name="$1"
    local source_dir="$2"
    local dest_dir="${SUPER_WS}/src/${package_name}"

    if [ ! -d "${source_dir}" ]; then
        fail "${package_name} source does not exist: ${source_dir}"
        return 1
    fi

    mkdir -p "${SUPER_WS}/src"

    if [ -e "${dest_dir}" ]; then
        local existing_resolved source_resolved
        existing_resolved="$(resolve_dir "${dest_dir}")"
        source_resolved="$(resolve_dir "${source_dir}")"
        if [ "${existing_resolved}" = "${source_resolved}" ]; then
            ok "${package_name} already linked at ${dest_dir}"
            return 0
        fi
        fail "${dest_dir} already exists but does not point to ${source_dir}; please move or fix it manually"
        return 1
    fi

    ln -s "${source_dir}" "${dest_dir}"
    ok "linked ${package_name}: ${dest_dir} -> ${source_dir}"
}

find_px4_dir() {
    if [ -n "${PX4_DIR:-}" ]; then
        if [ -d "${PX4_DIR}" ]; then
            readlink -f "${PX4_DIR}"
            return 0
        fi
        fail "PX4_DIR is set but does not exist: ${PX4_DIR}"
        return 1
    fi

    local candidate
    for candidate in \
        "${HOME}/PX4-Autopilot" \
        "${HOME}/ws/PX4-Autopilot" \
        "${HOME}/src/PX4-Autopilot" \
        "${HOME}/super_ws/src/PX4-Autopilot"; do
        if [ -d "${candidate}" ]; then
            readlink -f "${candidate}"
            return 0
        fi
    done

    fail "PX4-Autopilot not found. Please clone/build PX4 or run with PX4_DIR=/path/to/PX4-Autopilot."
    return 1
}

find_px4ctrl_dir() {
    local candidate
    for candidate in \
        "${SUPER_DRONE_DIR}/realflight_modules/px4ctrl" \
        "${HOME}/ws/SUPER_DRONE/realflight_modules/px4ctrl" \
        "${HOME}/REAL_DRONE_400/src/px4ctrl"; do
        if [ -d "${candidate}" ]; then
            readlink -f "${candidate}"
            return 0
        fi
    done
    return 1
}

echo "[preflight] SUPER_WS=${SUPER_WS}"
echo "[preflight] SUPER_DRONE_DIR=${SUPER_DRONE_DIR}"
echo "[preflight] GEZOGO_DIR=${GEZOGO_DIR}"

PX4_FOUND_DIR="$(find_px4_dir || true)"
if [ -n "${PX4_FOUND_DIR}" ]; then
    PX4_DIR="${PX4_FOUND_DIR}"
    PX4_STATUS="OK"
    ok "PX4_DIR=${PX4_DIR}"
else
    FAIL=1
fi

if [ -f "${GEOID_PATH}" ]; then
    GEOGRAPHICLIB_STATUS="OK"
    ok "GeographicLib geoid dataset found: ${GEOID_PATH}"
else
    echo "WARN: GeographicLib geoid dataset is missing: ${GEOID_PATH}" >&2
    echo "      Install it with one of:" >&2
    echo "        sudo apt install geographiclib-tools" >&2
    echo "        sudo geographiclib-get-geoids egm96-5" >&2
    echo "      or:" >&2
    echo "        sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh" >&2
    echo "      To let this preflight try sudo installation, rerun with INSTALL_GEOGRAPHICLIB=1." >&2
    if [ "${INSTALL_GEOGRAPHICLIB:-0}" = "1" ]; then
        if command -v geographiclib-get-geoids >/dev/null 2>&1; then
            sudo geographiclib-get-geoids egm96-5 || fail "geographiclib-get-geoids egm96-5 failed"
        elif [ -x /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh ]; then
            sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh || fail "MAVROS GeographicLib dataset installer failed"
        else
            sudo apt install geographiclib-tools && sudo geographiclib-get-geoids egm96-5 || fail "GeographicLib installation failed"
        fi
        if [ -f "${GEOID_PATH}" ]; then
            GEOGRAPHICLIB_STATUS="OK"
            ok "GeographicLib geoid dataset installed"
        fi
    fi
fi

NATIONALS_SIM_SRC="${GEZOGO_DIR}/gazebo_px4_nationals/nationals_sim"
if link_package "nationals_sim" "${NATIONALS_SIM_SRC}"; then
    NATIONALS_SIM_STATUS="OK"
fi

PX4CTRL_SRC="$(find_px4ctrl_dir || true)"
if [ -n "${PX4CTRL_SRC}" ]; then
    if link_package "px4ctrl" "${PX4CTRL_SRC}"; then
        PX4CTRL_STATUS="OK"
    fi
else
    fail "px4ctrl not found. Expected one of: ${SUPER_DRONE_DIR}/realflight_modules/px4ctrl, ${HOME}/ws/SUPER_DRONE/realflight_modules/px4ctrl, ${HOME}/REAL_DRONE_400/src/px4ctrl."
fi

if [ ! -f "${LAYOUT_PATH}" ] || [ ! -f "${WORLD_PATH}" ]; then
    if [ -f "${GENERATOR}" ]; then
        echo "[preflight] generating nationals world/layout seed ${SEED}"
        python3 "${GENERATOR}" --seed "${SEED}" || fail "world/layout generation failed"
    else
        fail "world/layout missing and generator is not found: ${GENERATOR}"
    fi
else
    ok "nationals world/layout exist for seed ${SEED}"
fi

if [ -d "${SUPER_WS}" ] && source_ros; then
    echo "[preflight] catkin_make -DCMAKE_BUILD_TYPE=Release"
    if (cd "${SUPER_WS}" && catkin_make -DCMAKE_BUILD_TYPE=Release); then
        CATKIN_STATUS="OK"
        ok "catkin_make completed"
        if [ -f "${SUPER_WS}/devel/setup.bash" ]; then
            # shellcheck disable=SC1091
            set +u
            source "${SUPER_WS}/devel/setup.bash"
            set -u
        else
            fail "catkin_make completed but ${SUPER_WS}/devel/setup.bash was not found"
            CATKIN_STATUS="FAIL"
        fi
    else
        CATKIN_STATUS="FAIL"
        fail "catkin_make failed"
    fi
else
    CATKIN_STATUS="FAIL"
    fail "cannot build because SUPER_WS or ROS setup is missing"
fi

echo
echo "[preflight] rospack checks"
for package in nationals_sim px4ctrl mission_planner super_planner quadrotor_msgs; do
    if rospack find "${package}" >/dev/null 2>&1; then
        echo "OK: rospack find ${package} -> $(rospack find "${package}")"
    else
        echo "FAIL: rospack find ${package}" >&2
        FAIL=1
    fi
done

echo
echo "[preflight] summary"
echo "SUPER_WS=${SUPER_WS}"
echo "SUPER_DRONE_DIR=${SUPER_DRONE_DIR}"
echo "GEZOGO_DIR=${GEZOGO_DIR}"
echo "PX4_DIR=${PX4_DIR:-missing}"
echo "GeographicLib=${GEOGRAPHICLIB_STATUS}"
echo "nationals_sim=${NATIONALS_SIM_STATUS}"
echo "px4ctrl=${PX4CTRL_STATUS}"
echo "catkin_make=${CATKIN_STATUS}"
echo
echo "[preflight] next steps"
echo "  1. ./scripts/run_nationals_px4_sitl_world.sh"
echo "  2. ./scripts/run_nationals_mavros.sh"
echo "  3. roslaunch nationals_sim nationals_nodes.launch layout_file:=${LAYOUT_PATH} start_mission_driver:=false"
echo "  4. ./scripts/run_nationals_super_sitl_smoke.sh"
echo "  5. ./scripts/run_nationals_px4ctrl_sitl.sh"
echo "  6. ./scripts/check_nationals_px4_sitl_topics.sh"
echo
echo "SITL only. Do not arm, take off, land, install propellers, or use this workflow on real hardware."

if [ "${FAIL}" -eq 0 ]; then
    echo "PASS: nationals PX4 SITL environment preflight completed"
    exit 0
fi

echo "FAIL: nationals PX4 SITL environment preflight found issues" >&2
exit 1

#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${WORKSPACE:-${HOME}/super_ws}}"
TMP_LAUNCH="${TMPDIR:-/tmp}/nationals_px4ctrl_sitl_$$.launch"
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

PX4CTRL_DIR="$(rospack find px4ctrl 2>/dev/null || true)"
if [ -z "${PX4CTRL_DIR}" ]; then
    echo "FAIL: ROS package px4ctrl is not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first." >&2
    exit 1
fi

RUN_CTRL_LAUNCH="${PX4CTRL_DIR}/launch/run_ctrl.launch"
if [ ! -f "${RUN_CTRL_LAUNCH}" ]; then
    echo "FAIL: px4ctrl launch file not found: ${RUN_CTRL_LAUNCH}" >&2
    exit 1
fi

cleanup() {
    rm -f "${TMP_LAUNCH}"
}
trap cleanup EXIT

if grep -Eq '<arg[[:space:]][^>]*name="no_RC"|<arg[[:space:]][^>]*name='\''no_RC'\''' "${RUN_CTRL_LAUNCH}"; then
    cat > "${TMP_LAUNCH}" <<'EOF'
<launch>
    <arg name="no_RC" default="true" />
    <param name="/px4ctrl/no_RC" value="true" type="bool" />
    <param name="/no_RC" value="true" type="bool" />
    <include file="$(find px4ctrl)/launch/run_ctrl.launch">
        <arg name="no_RC" value="$(arg no_RC)" />
    </include>
</launch>
EOF
else
    cat > "${TMP_LAUNCH}" <<'EOF'
<launch>
    <param name="/px4ctrl/no_RC" value="true" type="bool" />
    <param name="/no_RC" value="true" type="bool" />
    <include file="$(find px4ctrl)/launch/run_ctrl.launch" />
</launch>
EOF
    echo "WARN: run_ctrl.launch does not declare a no_RC arg; setting /px4ctrl/no_RC and /no_RC ROS params only." >&2
fi

echo "[nationals_px4ctrl_sitl] starting px4ctrl for PX4 SITL with no_RC=true"
echo "[nationals_px4ctrl_sitl] SITL only: this script does not arm, take off, land, or target real hardware"
exec roslaunch "${TMP_LAUNCH}"

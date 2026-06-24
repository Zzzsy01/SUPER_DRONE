#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${HOME}/super_ws}"
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14557}"
GEOID_PATH="/usr/share/GeographicLib/geoids/egm96-5.pgm"
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

if [ ! -f "${GEOID_PATH}" ]; then
    echo "WARN: GeographicLib geoid dataset is missing: ${GEOID_PATH}" >&2
    echo "      MAVROS may fail. Install with:" >&2
    echo "        sudo apt install geographiclib-tools" >&2
    echo "        sudo geographiclib-get-geoids egm96-5" >&2
    echo "      or:" >&2
    echo "        sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh" >&2
fi

echo "[nationals_mavros] starting MAVROS for PX4 SITL"
echo "[nationals_mavros] fcu_url=${FCU_URL}"
echo "[nationals_mavros] SITL only: this script does not arm, take off, land, or target real hardware"
exec roslaunch mavros px4.launch fcu_url:="${FCU_URL}"

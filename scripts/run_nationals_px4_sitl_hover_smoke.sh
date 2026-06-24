#!/usr/bin/env bash
set -euo pipefail

SUPER_WS="${SUPER_WS:-${HOME}/super_ws}"
SUPER_DRONE_DIR="${SUPER_DRONE_DIR:-${SUPER_WS}/src/SUPER_DRONE}"
TARGET_ALT="${TARGET_ALT:-1.0}"
HOVER_SECONDS="${HOVER_SECONDS:-8.0}"
BAG_DIR="${BAG_DIR:-${SUPER_DRONE_DIR}/logs}"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
mkdir -p "${ROS_HOME}" "${BAG_DIR}"

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

for package in mavros px4ctrl quadrotor_msgs; do
    if ! rospack find "${package}" >/dev/null 2>&1; then
        echo "FAIL: ROS package ${package} is not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first." >&2
        exit 1
    fi
done

if ! rostopic list >/dev/null 2>&1; then
    echo "FAIL: roscore is not reachable. Start PX4 SITL, MAVROS, and px4ctrl SITL first." >&2
    exit 1
fi

if ! timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
    echo "FAIL: /mavros/state is not connected=True. Start PX4 SITL and MAVROS first." >&2
    exit 1
fi

PX4CTRL_INFO="$(rostopic info /mavros/setpoint_raw/attitude 2>/dev/null || true)"
if ! echo "${PX4CTRL_INFO}" | grep -q "/px4ctrl"; then
    echo "FAIL: /mavros/setpoint_raw/attitude has no /px4ctrl publisher. Start ./scripts/run_nationals_px4ctrl_sitl.sh first." >&2
    exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${BAG_DIR}/nationals_px4_sitl_hover_smoke_${STAMP}.bag"
ROS_BAG_PID=""
ODOM_RELAY_PID=""

cleanup() {
    if [ -n "${ODOM_RELAY_PID}" ] && kill -0 "${ODOM_RELAY_PID}" >/dev/null 2>&1; then
        kill -INT "${ODOM_RELAY_PID}" >/dev/null 2>&1 || true
        wait "${ODOM_RELAY_PID}" >/dev/null 2>&1 || true
    fi
    if [ -n "${ROS_BAG_PID}" ] && kill -0 "${ROS_BAG_PID}" >/dev/null 2>&1; then
        kill -INT "${ROS_BAG_PID}" >/dev/null 2>&1 || true
        wait "${ROS_BAG_PID}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

if timeout 3 rostopic echo -n 1 /Odom_high_freq >/dev/null 2>&1; then
    echo "[hover_smoke] /Odom_high_freq already has messages"
else
    echo "[hover_smoke] starting odom relay: /mavros/local_position/odom -> /Odom_high_freq"
    rosrun topic_tools relay /mavros/local_position/odom /Odom_high_freq __name:=nationals_hover_odom_relay &
    ODOM_RELAY_PID="$!"
    sleep 1
fi

if ! timeout 5 rostopic echo -n 1 /Odom_high_freq >/dev/null 2>&1; then
    echo "FAIL: /Odom_high_freq has no messages. px4ctrl needs this odom topic before hover smoke." >&2
    exit 1
fi

echo "[hover_smoke] SITL only. Do not use on real hardware."
echo "[hover_smoke] target_alt=${TARGET_ALT} hover_seconds=${HOVER_SECONDS}"
echo "[hover_smoke] recording bag: ${BAG_PATH}"
rosbag record -O "${BAG_PATH}" --lz4 \
    /mavros/state \
    /mavros/local_position/odom \
    /position_cmd \
    /mavros/setpoint_raw/attitude &
ROS_BAG_PID="$!"
sleep 1

python3 "${SUPER_DRONE_DIR}/scripts/nationals_px4_sitl_hover_smoke.py" \
    _target_alt:="${TARGET_ALT}" \
    _hover_seconds:="${HOVER_SECONDS}"

echo "[hover_smoke] bag recorded: ${BAG_PATH}"

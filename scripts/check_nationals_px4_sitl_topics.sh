#!/usr/bin/env bash
set -u
set -o pipefail

SUPER_WS="${SUPER_WS:-${HOME}/super_ws}"
FAIL=0
HZ_TIMEOUT="${HZ_TIMEOUT:-6}"
export ROS_HOME="${ROS_HOME:-/tmp/super_drone_ros_home}"
mkdir -p "${ROS_HOME}"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "FAIL: /opt/ros/noetic/setup.bash not found"
    exit 1
fi
if [ ! -f "${SUPER_WS}/devel/setup.bash" ]; then
    echo "FAIL: ${SUPER_WS}/devel/setup.bash not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first."
    exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${SUPER_WS}/devel/setup.bash"
set -u

for package in nationals_sim px4ctrl mission_planner super_planner quadrotor_msgs; do
    if ! rospack find "${package}" >/dev/null 2>&1; then
        echo "FAIL: ROS package ${package} is not found. Run ./scripts/preflight_nationals_px4_sitl_env.sh first."
        exit 1
    fi
done

topic_exists() {
    rostopic list 2>/dev/null | grep -qx "$1"
}

check_exists() {
    local topic="$1"
    if topic_exists "${topic}"; then
        echo "OK: ${topic} exists"
    else
        echo "FAIL: ${topic} missing"
        FAIL=1
    fi
}

check_hz() {
    local topic="$1"
    echo "[check] rostopic hz ${topic}"
    if timeout "${HZ_TIMEOUT}" rostopic hz "${topic}"; then
        echo "OK: ${topic} has frequency"
    else
        echo "FAIL: ${topic} has no measurable frequency"
        FAIL=1
    fi
}

if ! rostopic list >/dev/null 2>&1; then
    echo "FAIL: roscore is not reachable. Start PX4 SITL, MAVROS, SUPER, and px4ctrl smoke nodes first."
    exit 1
fi

check_exists "/mavros/state"
if topic_exists "/mavros/state"; then
    if timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
        echo "OK: /mavros/state connected=True"
    else
        echo "FAIL: /mavros/state is not connected=True"
        FAIL=1
    fi
fi

for topic in \
    "/Odom_high_freq" \
    "/cloud_registered" \
    "/position_cmd" \
    "/mavros/setpoint_raw/attitude"; do
    if topic_exists "${topic}"; then
        check_hz "${topic}"
    else
        echo "FAIL: ${topic} missing"
        FAIL=1
    fi
done

if topic_exists "/position_cmd"; then
    echo "[check] rostopic info /position_cmd"
    POSITION_INFO="$(rostopic info /position_cmd 2>/dev/null || true)"
    echo "${POSITION_INFO}"
    if echo "${POSITION_INFO}" | grep -Eiq 'Subscribers:.*[1-9]|px4ctrl'; then
        if echo "${POSITION_INFO}" | grep -Eiq 'px4ctrl'; then
            echo "OK: /position_cmd has a px4ctrl subscriber"
        else
            echo "FAIL: /position_cmd has subscribers, but px4ctrl was not visible by name"
            FAIL=1
        fi
    else
        echo "FAIL: /position_cmd has no px4ctrl subscriber"
        FAIL=1
    fi
fi

if rosmsg show quadrotor_msgs/PositionCommand >/dev/null 2>&1; then
    echo "OK: quadrotor_msgs/PositionCommand is available"
else
    echo "FAIL: quadrotor_msgs/PositionCommand is not available to rosmsg"
    FAIL=1
fi

echo "INFO: real Mid-360S, real FAST-LIO, and real flight controller are intentionally not required."
echo "INFO: SITL only. This checker does not arm, take off, land, or target real hardware."

if [ "${FAIL}" -eq 0 ]; then
    echo "PASS: nationals PX4 SITL smoke topics look connected"
    exit 0
fi

echo "FAIL: nationals PX4 SITL smoke topic check failed"
exit 1

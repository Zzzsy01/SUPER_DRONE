#!/usr/bin/env bash
set -u

POINTS_CANDIDATES=("${MID360S_POINTS_TOPIC:-}" "/livox/lidar" "/livox/pointcloud" "/livox/lidar/pointcloud")
IMU_CANDIDATES=("${MID360S_IMU_TOPIC:-}" "/livox/imu")
FAIL=0

topic_exists() {
    rostopic list 2>/dev/null | grep -qx "$1"
}

first_existing_topic() {
    local topic
    for topic in "$@"; do
        [ -z "${topic}" ] && continue
        if topic_exists "${topic}"; then
            echo "${topic}"
            return 0
        fi
    done
    return 1
}

check_hz() {
    local topic="$1"
    echo "[check] rostopic hz ${topic}"
    timeout 6 rostopic hz "${topic}" || return 1
}

if ! rostopic list >/dev/null 2>&1; then
    echo "FAIL: roscore is not reachable. Start roscore or the ROS launch stack first."
    exit 1
fi

POINTS_TOPIC="$(first_existing_topic "${POINTS_CANDIDATES[@]}")" || {
    echo "FAIL: Mid-360S driver topic missing. Check driver launch/config."
    FAIL=1
}

IMU_TOPIC="$(first_existing_topic "${IMU_CANDIDATES[@]}")" || {
    echo "WARN: Mid-360S IMU topic not found. Set MID360S_IMU_TOPIC if your driver uses another name."
}

if [ -n "${POINTS_TOPIC:-}" ]; then
    echo "OK: Mid-360S points topic: ${POINTS_TOPIC}"
    check_hz "${POINTS_TOPIC}" || FAIL=1
fi

if [ -n "${IMU_TOPIC:-}" ]; then
    echo "OK: Mid-360S IMU topic: ${IMU_TOPIC}"
    check_hz "${IMU_TOPIC}" || true
fi

if topic_exists "/cloud_registered"; then
    echo "OK: FAST-LIO cloud_registered exists"
    check_hz "/cloud_registered" || FAIL=1
else
    echo "FAIL: FAST-LIO cloud_registered missing. Check FAST-LIO input topic remap."
    FAIL=1
fi

if topic_exists "/Odom_high_freq"; then
    echo "OK: FAST-LIO odom exists"
    check_hz "/Odom_high_freq" || FAIL=1
else
    echo "FAIL: FAST-LIO odom missing. Check FAST-LIO status."
    FAIL=1
fi

if [ "${FAIL}" -eq 0 ]; then
    echo "PASS: Mid-360S and FAST-LIO topics look reachable"
    exit 0
fi

echo "FAIL: Mid-360S / FAST-LIO topic check failed"
exit 1

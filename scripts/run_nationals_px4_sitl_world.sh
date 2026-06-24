#!/usr/bin/env bash
set -euo pipefail

GUOSAI_REPO="${GUOSAI_REPO:-${HOME}/ws/gezogo-guosai}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
SEED="${SEED:-2026}"
GENERATED_DIR="${GUOSAI_REPO}/gazebo_px4_nationals/generated/seed_${SEED}"
WORLD_PATH="${GENERATED_DIR}/nationals_field.world"
LAYOUT_PATH="${GENERATED_DIR}/layout.json"
GENERATOR="${GUOSAI_REPO}/tools/generate_nationals_world.py"

if [ ! -d "${PX4_DIR}" ]; then
    echo "FAIL: PX4_DIR does not exist: ${PX4_DIR}" >&2
    exit 1
fi

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
echo "[nationals_px4_sitl] this script does not start SUPER, px4ctrl, MAVROS, arm, takeoff, land, or mission goals"

export PX4_SITL_WORLD="${WORLD_PATH}"
exec make -C "${PX4_DIR}" px4_sitl_default gazebo-classic_iris

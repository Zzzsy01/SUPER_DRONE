#!/usr/bin/env bash
set -euo pipefail

echo "[demo_full_route] DEMO ONLY: Gazebo/PX4 SITL display and recording route."
echo "[demo_full_route] Not for real hardware and not a strict competition validation."

export STAGE=demo_full_route
export HEADLESS="${HEADLESS:-1}"
export TIMEOUT_SEC="${TIMEOUT_SEC:-360}"
export SITL_DEMO_ALT="${SITL_DEMO_ALT:-1.30}"
export SITL_DEMO_SPACING="${SITL_DEMO_SPACING:-0.60}"
export SITL_DEMO_SWITCH_RADIUS="${SITL_DEMO_SWITCH_RADIUS:-1.00}"
export SITL_DEMO_INCLUDE_BOUNDARY_WALLS="${SITL_DEMO_INCLUDE_BOUNDARY_WALLS:-false}"

exec "$(dirname "$0")/run_nationals_px4_sitl_full_mission.sh"

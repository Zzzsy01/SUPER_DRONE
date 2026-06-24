#!/usr/bin/env bash
set -euo pipefail

FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14557}"

echo "[nationals_mavros] starting MAVROS for PX4 SITL"
echo "[nationals_mavros] fcu_url=${FCU_URL}"
exec roslaunch mavros px4.launch fcu_url:="${FCU_URL}"

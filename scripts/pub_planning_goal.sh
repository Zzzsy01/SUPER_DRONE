#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 X Y Z"
    echo "Example: $0 8.0 0.0 1.0"
    exit 2
fi

if ! command -v rostopic >/dev/null 2>&1; then
    echo "ERROR: rostopic not found. Source the ROS1 workspace first."
    exit 1
fi

if ! rostopic list >/dev/null 2>&1; then
    echo "ERROR: roscore is not reachable. Start roscore or roslaunch first."
    exit 1
fi

X="$1"
Y="$2"
Z="$3"

echo "Publishing /planning/click_goal in frame world: x=${X}, y=${Y}, z=${Z}"
rostopic pub -1 /planning/click_goal geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position:
    x: ${X}
    y: ${Y}
    z: ${Z}
  orientation:
    w: 1.0"

#!/usr/bin/env bash
set -u

SESSION="${SESSION:-real_check}"
REAL_WS="${REAL_WS:-${HOME}/REAL_DRONE_400}"
SUPER_WS="${SUPER_WS:-${HOME}/super_ws}"
MID360S_START_CMD="${MID360S_START_CMD:-./start_lidar.sh}"
FAST_LIO_LAUNCH_CMD="${FAST_LIO_LAUNCH_CMD:-roslaunch fast_lio mapping_mid360.launch}"
MAVROS_CMD="${MAVROS_CMD:-roslaunch mavros px4.launch}"
PX4CTRL_CMD="${PX4CTRL_CMD:-roslaunch px4ctrl run_ctrl.launch}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is not installed"
    exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux attach -t "${SESSION}"
    exit 0
fi

tmux new-session -d -s "${SESSION}" -n real
tmux send-keys -t "${SESSION}:real.0" "cd ${REAL_WS} && source devel/setup.bash && ${MID360S_START_CMD}" C-m

tmux split-window -h -t "${SESSION}:real"
tmux send-keys -t "${SESSION}:real.1" "cd ${REAL_WS} && source devel/setup.bash && ${MAVROS_CMD}" C-m

tmux split-window -v -t "${SESSION}:real.0"
tmux send-keys -t "${SESSION}:real.2" "cd ${REAL_WS} && source devel/setup.bash && ${FAST_LIO_LAUNCH_CMD}" C-m

tmux split-window -v -t "${SESSION}:real.1"
tmux send-keys -t "${SESSION}:real.3" "cd ${REAL_WS} && source devel/setup.bash && ${PX4CTRL_CMD}" C-m

tmux new-window -t "${SESSION}" -n super
tmux send-keys -t "${SESSION}:super.0" "cd ${SUPER_WS} && source devel/setup.bash && roslaunch mission_planner real_competition.launch" C-m

tmux split-window -h -t "${SESSION}:super"
tmux send-keys -t "${SESSION}:super.1" "cd ${SUPER_WS} && source devel/setup.bash && roslaunch mission_planner real_competition_rviz.launch" C-m

tmux split-window -v -t "${SESSION}:super.1"
tmux send-keys -t "${SESSION}:super.2" "cd ${SUPER_WS} && source devel/setup.bash && echo 'Press Enter after manual approval to record:' && read && rosbag record -O real_check.bag --lz4 /Odom_high_freq /cloud_registered /position_cmd /planning/click_goal /mavros/state /mavros/local_position/odom /mavros/setpoint_raw/attitude" C-m

tmux new-window -t "${SESSION}" -n monitor
tmux send-keys -t "${SESSION}:monitor.0" "cd ${SUPER_WS} && source devel/setup.bash && while true; do clear; rostopic list | grep -E 'livox|cloud_registered|Odom_high_freq|position_cmd|mavros|px4ctrl'; echo; timeout 4 rostopic hz /cloud_registered; timeout 4 rostopic hz /Odom_high_freq; timeout 4 rostopic hz /position_cmd; sleep 2; done" C-m

tmux select-window -t "${SESSION}:real"
tmux attach -t "${SESSION}"

#!/usr/bin/env bash
set -u

SESSION="${SESSION:-gazebo_planning}"
WORKSPACE="${HOME}/super_ws"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is not installed"
    exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux attach -t "${SESSION}"
    exit 0
fi

tmux new-session -d -s "${SESSION}" -n sim
tmux send-keys -t "${SESSION}:sim" "cd ${WORKSPACE} && source devel/setup.bash && roslaunch mission_planner gazebo_planning_sim.launch gui:=true rviz:=true" C-m

tmux split-window -h -t "${SESSION}:sim"
tmux send-keys -t "${SESSION}:sim.1" "cd ${WORKSPACE} && source devel/setup.bash && watch -n 1 \"rostopic list | grep -E 'cloud_registered|Odom_high_freq|position_cmd|planning/click_goal|mock_drone|competition_sim'\"" C-m

tmux split-window -v -t "${SESSION}:sim.1"
tmux send-keys -t "${SESSION}:sim.2" "cd ${WORKSPACE} && source devel/setup.bash && while true; do rostopic hz /cloud_registered /Odom_high_freq /position_cmd; sleep 2; done" C-m

tmux select-pane -t "${SESSION}:sim.0"
tmux attach -t "${SESSION}"

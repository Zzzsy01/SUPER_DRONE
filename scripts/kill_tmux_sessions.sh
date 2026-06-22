#!/usr/bin/env bash
set -u

for session in gazebo_planning real_check; do
    if tmux has-session -t "${session}" 2>/dev/null; then
        tmux kill-session -t "${session}"
        echo "Killed tmux session: ${session}"
    else
        echo "No tmux session: ${session}"
    fi
done

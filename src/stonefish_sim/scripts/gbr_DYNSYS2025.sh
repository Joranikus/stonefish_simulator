#!/bin/bash

SESSION="gbr_DYNSYS2025"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n main
tmux split-window -h -t "$SESSION:0"
tmux split-window -h -t "$SESSION:0.1"

KILL_CMD="tmux kill-session -t $SESSION"

tmux send-keys -t "$SESSION:0.0" "
bash -i -c '
  trap \"$KILL_CMD\" EXIT
  source install/setup.bash
  ros2 launch stonefish_sim simulation.launch.py scenario:=gbr_DYNSYS2025 rendering_quality:=low
'
" C-m

tmux send-keys -t "$SESSION:0.1" "
bash -i -c '
  source install/setup.bash
  ros2 service call /stonefish_ros2/enable_currents std_srvs/srv/Trigger
'
" C-m


tmux send-keys -t "$SESSION:0.2" "
bash -i -c '
  trap \"$KILL_CMD\" EXIT
  source install/setup.bash
  ros2 run gbr_manual_keyboard_controller gbr_manual_keyboard_controller
'
" C-m

tmux attach-session -t "$SESSION"
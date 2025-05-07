#!/bin/bash

SESSION="gbr_demo"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n main
tmux split-window -h -t "$SESSION:0"

KILL_CMD="tmux kill-session -t $SESSION"

tmux send-keys -t "$SESSION:0.0" "
bash -i -c '
  trap \"$KILL_CMD\" EXIT
  source install/setup.bash
  ros2 launch stonefish_sim simulation.launch.py scenario:=gbr_keyboard_demo rendering_quality:=low
'
" C-m

tmux send-keys -t "$SESSION:0.1" "
bash -i -c '
  trap \"$KILL_CMD\" EXIT
  source install/setup.bash
  ros2 run gbr_manual_keyboard_controller gbr_manual_keyboard_controller
'
" C-m

tmux attach-session -t "$SESSION"
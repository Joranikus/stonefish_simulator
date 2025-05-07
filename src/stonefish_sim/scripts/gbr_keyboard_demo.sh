#!/bin/bash

tmux new-session -d -s "gbr_demo"

tmux split-window -h -t "gbr_demo:0"

tmux send-keys -t "gbr_demo:0.0" "ros2 launch stonefish_sim simulation.launch.py scenario:=gbr_keyboard_demo window_res_x:=1920 window_res_y:=1080 rendering_quality:=low" C-m
tmux send-keys -t "gbr_demo:0.1" "ros2 run gbr_manual_keyboard_controller gbr_manual_keyboard_controller" C-m

tmux attach-session -t "gbr_demo"
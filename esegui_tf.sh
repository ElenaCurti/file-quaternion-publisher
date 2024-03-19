#!/bin/bash

# Create a new tmux session named 'my_session' with bash shell
tmux new-session -d -s my_session bash

# Split the window vertically to create three terminals
tmux split-window -v -t my_session:0 bash
tmux split-window -v -t my_session:0 bash

# Send commands to the first and second terminals
tmux send-keys -t my_session:0.0 "cd ~/mmr-drive  ; source install/local_setup.bash  ; ros2 launch fast_lio fast_lio_launch.py " C-m
tmux send-keys -t my_session:0.1 "cd ~/file-quaternion-publisher ; source install/local_setup.bash  ;  ros2 run file_quaternion_publisher publish_tf_transform f_tre_giri_7_marzo.txt " C-m
tmux send-keys -t my_session:0.2 "cd /mnt/hgfs/G/Elena ; ros2 bag play tre_giri_orario/ --topics /velodyne_points /imu/data  " C-m

# Attach to the session to view the terminals
tmux -2 attach-session -d -t my_session


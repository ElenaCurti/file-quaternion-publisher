#!/bin/bash

cleanup() {
    echo "Script interrupted. Cleaning up..."
    # Perform cleanup actions here
    # For example, you can kill the background processes
    kill $(jobs -p) &> /dev/null
    exit 1
}

trap cleanup SIGINT


# TODO cambia qui il path del file-quaternion-publisher 
cd ~/file-quaternion-publisher 

source install/local_setup.bash 
ros2 run file_quaternion_publisher publish_tf_orbslam f_tre_giri_7_marzo.txt &

# TODO cambia qui il path della repo 
cd ~/mmr-drive        

source install/local_setup.bash 
ros2 launch fast_lio fast_lio_launch.py &

# TODO cambia qui il path dove hai memorizzato la bag 
cd /mnt/hgfs/G/Elena ;     

ros2 bag play tre_giri_orario/ --topics /velodyne_points /imu/data  &

wait

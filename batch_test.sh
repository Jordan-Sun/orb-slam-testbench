#!/bin/bash
# Full batch script to run all in one script

# Usage function in case of incorrect arguments
usage() {
    echo "Usage: $0 <data_path> <num_iterations> [start_iteration]"
    exit 1
}

# Use the first argument as the base path to the data to collect
DATA_PATH=$1
cd $DATA_PATH
if [ $? -ne 0 ]; then
    echo "Error: Cannot change directory to $DATA_PATH"
    usage
fi

# Use the second argument as the number of iterations
NUM_ITERATIONS=$2
if ! [[ "$NUM_ITERATIONS" =~ ^[0-9]+$ ]]; then
    echo "Error: Number of iterations must be a positive integer"
    usage
fi

# Optionally start from the thrid argument if provided
if [ -n "$3" ]; then
    START_ITER=$3
    if ! [[ "$START_ITER" =~ ^[0-9]+$ ]]; then
        echo "Error: Start iteration must be a positive integer"
        usage
    fi
else
    START_ITER=1
fi

# Start ROS core in the background
export ROS_PACKAGE_PATH=/home/orb-slam-elastic/Examples_old/ROS:$ROS_PACKAGE_PATH
roscore &
ROSCORE_PID=$!

# Sleep for 5 seconds to ensure roscore starts properly
sleep 5

for i in $(seq 1 $NUM_ITERATIONS)
do
    # Create a directory for this iteration
    ITER_DIR="iteration_$i"
    mkdir -p $ITER_DIR
    cd $ITER_DIR
    # Start ORB SLAM in the background and pipe its output to a log file
    taskset -c 2 rosrun ORB_SLAM3 Stereo_Inertial /home/orb-slam-elastic/Vocabulary/ORBvoc.txt /home/orb-slam-elastic/Examples_old/Stereo-Inertial/EuRoC.yaml true &> orb_slam.log &
    ORB_PID=$!

    # Sleep for 5 seconds to ensure ORB SLAM starts properly
    sleep 5

    # Play the bag file, and kill the ORB SLAM process with SIGINT when done
    rosbag play /data/MH_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
    sleep 1
    kill -SIGINT $ORB_PID

    # Leave the directory
    cd ..
done

# Kill the roscore process
kill -SIGINT $ROSCORE_PID

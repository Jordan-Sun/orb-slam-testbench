#!/bin/bash

# Full batch script to run all in one script for trace-cmd.

# Set up ORB SLAM environment variables
export ORB_SLAM_PATH="$HOME/Documents/orb-slam-testbench"
export ROS_PACKAGE_PATH="$ORB_SLAM_PATH:$ROS_PACKAGE_PATH"
# Set up ros
source /opt/ros/noetic/setup.sh

# Run ORB SLAM
rosrun ORB_SLAM3 Stereo_Inertial $ORB_SLAM_PATH/Vocabulary/ORBvoc.txt $ORB_SLAM_PATH/Examples_old/Stereo-Inertial/EuRoC.yaml true
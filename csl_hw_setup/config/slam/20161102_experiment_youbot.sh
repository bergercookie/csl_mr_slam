#!/usr/bin/env bash

# Single robot SLAM
# source this file on both the ubuntu machine and youbot

export MR_USE_MULTIMASTER=0

export MR_IS_MULTIROBOT_GRAPHSLAM=0

export MR_ROBOT_ID=15
export MR_ROBOT_MODEL="youbot"

# Topics are to be recorded in the Ubuntu machine
export MR_RECORD_TOPICS=1

# Robot HW Drivers
# Select only one of the following
export MR_ROBOT_DRIVERS_USE_POULIAS=0
export MR_ROBOT_DRIVERS_USE_YOUBOT=1
export MR_ROBOT_DRIVERS_USE_ARIA=0
export MR_OUTPUT_MESSAGES_TO="screen"

# LaserScanner
export MR_USE_LASER=1

# [!] WARNING
# urg_node doesn't seem to work properly with the hokuyo laser scanners available in the Control Systems lab
# It is *strongly* advised to set this to 0
export MR_LASER_USE_URG_NODE_PKG=0 # Use the urg_node ROS pkg instead of the hokuyo_node

export MR_LASER_NAME="hokuyo"

export MR_LASER_PORT="/dev/ttyACM1"

# Option only available with the urg_node laserScans package - ignored otherwise
export MR_LASER_SKIP_NUM_MESSAGES="1"

# Camera
export MR_USE_ONBOARD_CAMERA=0
export MR_ONBOARD_CAMERA_PORT="dev/video0"

# Teleoperation
export MR_USE_JOYSTICK=1
export MR_JOYSTICK_PORT="/dev/input/js0"

# Marker IDs for  common origin and ground-truth paths
export MR_ORIGIN_MARKER_ID="mf7"
export MR_ROBOT_MARKER_ID="mf1" # Just for initializing the anchor_frame_ID

# if true, Odd Aruco markers are used for tracking the agents' ground truth
# paths while even aruco markers for inter-robot meetings
export MR_USE_ODD_ARUCO_MARKERS_FOR_GT=1

export MR_COMPUTE_GROUND_TRUTH=1

#!/usr/bin/env bash

# Single robot SLAM
# source this file on both the ubuntu machine and youbot

export MR_USE_MULTIMASTER=0

export MR_IS_MULTIROBOT_GRAPHSLAM=0
export MR_IS_SIMULATION=0

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
export MR_LASER_NAME="hokuyo"
export MR_LASER_USE_HOKUYO_NODE_PKG=0 # should we use the hokuyo_node pkg or the newer  urg_node?
export MR_LASER_PORT="/dev/ttyACM1"
export MR_LASER_SKIP_NUM_MESSAGES="2"

# Camera
export MR_USE_ONBOARD_CAMERA=0
export MR_ONBOARD_CAMERA_PORT="dev/video0"

# Teleoperation
export MR_USE_JOYSTICK=1
export MR_JOYSTICK_PORT="/dev/input/js0"

# Marker IDs for  common origin and ground-truth paths
export MR_ORIGIN_MARKER_ID="mf7"
export MR_ROBOT_MARKER_ID="mf0"

export MR_COMPUTE_GROUND_TRUTH=1

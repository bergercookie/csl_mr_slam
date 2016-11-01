#!/usr/bin/env bash

# Single robot SLAM 

export MR_IS_MULTIROBOT_GRAPHSLAM=0
export MR_IS_SIMULATION=0

export MR_ROBOT_ID=15
export MR_ROBOT_MODEL="youbot"

# Topics are to be recorded in the Ubuntu machine
export MR_RECORD_TOPICS=0

# Robot HW Drivers
# Select only one of the following
export MR_ROBOT_DRIVERS_USE_POULIAS=0
export MR_ROBOT_DRIVERS_USE_YOUBOT=1
export MR_ROBOT_DRIVERS_USE_ARIA=0
export MR_OUTPUT_MESSAGES_TO="screen"

# LaserScanner
export MR_USE_LASER=1
export MR_LASER_NAME="hokuyo"
export MR_LASER_PORT="/dev/ttyACM0"

# Camera
export MR_USE_CAMERA=0
export MR_CAMERA_PORT="dev/video0"

# Teleoperation
export MR_USE_JOYSTICK=0

# Marker IDs for  common origin and ground-truth paths
export MR_ORIGIN_MARKER_ID="mf7"
export MR_ROBOT_MARKER_ID="mf1"

export MR_USE_ROOF_CAMERAS=1

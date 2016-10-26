#!/usr/bin/env bash

# Fri Oct 21 21:45:41 EEST 2016, Nikos Koukis
# Setup the necessary for MR-SLAM environment variables in a compact way

export MR_ROBOT_ID=18
export MR_ROSBAG_DIR="${HOME}"
export MR_WLAN_INTERFACE="wlan6"
export MR_ROBOT_MODEL="pioneer_2at"

export MR_USE_SIM_TIME=0
export MR_USE_MULTIMASTER=0
export MR_RECORD_ALL_TOPICS=1

# Robot Model
# Select only one of the following
export MR_ROBOT_MODEL_IS_PIONEER_2AT=1
export MR_ROBOT_MODEL_IS_PIONEER_3DX=0
export MR_ROBOT_MODEL_IS_YOUBOT=0

# Robot HW Drivers
# Select only one of the following
export MR_ROBOT_DRIVERS_USE_POULIAS=1
export MR_ROBOT_DRIVERS_USE_YOUBOT=0
export MR_ROBOT_DRIVERS_USE_ARIA=0

export MR_USE_LASER=1

export MR_OUTPUT_MESSAGES_TO="screen"

# Application specific Environment Variables
export MR_LASER_NAME="hokuyo"
export MR_LASER_PORT="/dev/ttyACM1"
export MR_POULIAS_ARDUINO_PORT="/dev/ttyACM0"
export MR_POULIAS_JOYSTICK_PORT="/dev/input/js0"

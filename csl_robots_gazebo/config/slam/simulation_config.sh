#!/usr/bin/env bash

# Simulation-related variables are prefixed with SIMUL

# Setup one or two graphslam agents in the simulation
export MR_IS_MULTIROBOT_GRAPHSLAM=1

# All nodes read this variable and output the messages accordingly
export MR_OUTPUT_MESSAGES_TO="screen"

# Open the rqt robot steering windows.
export MR_USE_RQT_ROBOT_STEERING=1

# TODO - fill in the necessary variables

# Syntax for eaach robot properties
# MR_ROBOT_X_PROPERTYNAME=PROPERTY_VALUE

# MR_ROBOT_1
######################

# use different xacro model based on the ROBOT_X_MODEL variable
# Available models:
# pioneer_3at
# youbot - TODO
# pioneer_3dx - TODO
# pioneer_2dx - TODO
export MR_ROBOT_1_MODEL="pioneer_3at"

# define the robot name + corresponding namespace
export MR_ROBOT_1_NAME="pioneer_3at_1"

export MR_ROBOT_1_POS_X=-1.49
export MR_ROBOT_1_POS_Y=-3
export MR_ROBOT_1_POS_Z=0.051

export MR_ROBOT_1_ROT_X=0
export MR_ROBOT_1_ROT_Y=0
export MR_ROBOT_1_ROT_Z=0

# MR_ROBOT 2
######################
export MR_ROBOT_2_MODEL="pioneer_3at"
export MR_ROBOT_2_NAME="pioneer_3at_2"

export MR_ROBOT_2_POS_X=10
export MR_ROBOT_2_POS_Y=-6
export MR_ROBOT_2_POS_Z=0.051

export MR_ROBOT_2_ROT_X=0
export MR_ROBOT_2_ROT_Y=0
export MR_ROBOT_2_ROT_Z=3.1415

# TODO
# define the robot name + corresponding namespace
export MR_ROBOT_3_MODEL="pioneer_3at"
#export MR_ROBOT_3_NAME="pioneer_3at_3"

export MR_ROBOT_3_POS_X=-1.49
export MR_ROBOT_3_POS_Y=-6
export MR_ROBOT_3_POS_Z=0.051

export MR_ROBOT_3_ROT_X=0
export MR_ROBOT_3_ROT_Y=0
export MR_ROBOT_3_ROT_Z=0



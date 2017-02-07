#!/usr/bin/env bash

# Computer hostname
export MR_HOSTNAME="$(hostname)"
# Last field of the IP of the computer that runs the simulation
# e.g. 147.102.51.142 => 142
# Used as a suffix in the name of the multimaster configuration
export MR_IP_LAST_FIELD="$(ifconfig eth0 | awk '/inet addr/{print substr($2,6)}' | cut -d "." -f4)"

# Setup one or two graphslam agents in the simulation
export MR_IS_MULTIROBOT_GRAPHSLAM=1

# All nodes read this variable and output the messages accordingly
export MR_OUTPUT_MESSAGES_TO="screen"

# Global coordinates frame ID
export MR_GLOBAL_FRAME_ID="map"

# Open the rqt robot steering windows.
export MR_USE_RQT_ROBOT_STEERING=1

# Launch the graphSLAM-related nodes in differeent namespaces
export MR_USE_DIFFERENT_ROSCORES=1

# TODO - fill in the necessary variables

# Syntax for each robot properties
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

# define the robot name. This will also be the corresponding namespace
export MR_ROBOT_1_NAME="${MR_HOSTNAME}_${MR_IP_LAST_FIELD}"

export MR_ROBOT_1_POS_X=-1.5
export MR_ROBOT_1_POS_Y=-3
export MR_ROBOT_1_POS_Z=0.051

export MR_ROBOT_1_ROT_X=0
export MR_ROBOT_1_ROT_Y=0
export MR_ROBOT_1_ROT_Z=0


# TODO - runtime error if I use CFixedIntervalsNRD here...
export MR_ROBOT_1_NRD="CICPCriteriaNRD_CM"
export MR_ROBOT_1_ERD="CLoopCloserERD_CM"
export MR_ROBOT_1_GSO="CLevMarqGSO"

# MR_ROBOT 2
######################
#export MR_ROBOT_2_MODEL="pioneer_3at"
#export MR_ROBOT_2_NAME="${MR_HOSTNAME}_11312_${MR_IP_LAST_FIELD}"

export MR_ROBOT_2_POS_X=10
export MR_ROBOT_2_POS_Y=-6
export MR_ROBOT_2_POS_Z=0.051

export MR_ROBOT_2_ROT_X=0
export MR_ROBOT_2_ROT_Y=0
export MR_ROBOT_2_ROT_Z=3.1415

export MR_ROBOT_2_NRD="CICPCriteriaNRD_CM"
export MR_ROBOT_2_ERD="CLoopCloserERD_CM"
export MR_ROBOT_2_GSO="CLevMarqGSO"

###########################################3
# TODO - Run simulation with 3 robots

# define the robot name + corresponding namespace
export MR_ROBOT_3_MODEL="pioneer_3at"
export MR_ROBOT_3_NAME="${MR_HOSTNAME}_11312_${MR_IP_LAST_FIELD}"

export MR_ROBOT_3_POS_X=-1.5
export MR_ROBOT_3_POS_Y=-5.2
export MR_ROBOT_3_POS_Z=0.051

export MR_ROBOT_3_ROT_X=0
export MR_ROBOT_3_ROT_Y=0
export MR_ROBOT_3_ROT_Z=0

export MR_ROBOT_3_NRD="CICPCriteriaNRD_CM"
export MR_ROBOT_3_ERD="CLoopCloserERD_CM"
export MR_ROBOT_3_GSO="CLevMarqGSO"


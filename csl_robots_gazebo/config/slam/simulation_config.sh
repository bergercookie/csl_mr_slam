#!/usr/bin/env bash

# Computer hostname
export MR_HOSTNAME="$(hostname)"
# Last field of the IP of the computer that runs the simulation
# e.g. 147.102.51.142 => 142
# Used as a suffix in the name of the multimaster configuration
export MR_IP_LAST_FIELD="$(ifconfig wlan0 | awk '/inet addr/{print substr($2,6)}' | cut -d "." -f4)"

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

export MR_ROBOT_1_NRD="CICPCriteriaNRD_CM"
export MR_ROBOT_1_ERD="CLoopCloserERD_CM"
export MR_ROBOT_1_GSO="CLevMarqGSO"

# MR_ROBOT 2
######################
export MR_ROBOT_2_MODEL="pioneer_3at"
export MR_ROBOT_2_NAME="${MR_HOSTNAME}_11312_${MR_IP_LAST_FIELD}"


export MR_ROBOT_2_NRD="CICPCriteriaNRD_CM"
#export MR_ROBOT_2_NRD="CFixedIntervalsNRD_CM"
export MR_ROBOT_2_ERD="CLoopCloserERD_CM"
export MR_ROBOT_2_GSO="CLevMarqGSO"

###########################################3
# TODO - Run simulation with 3 robots

# define the robot name + corresponding namespace
#export MR_ROBOT_3_MODEL="pioneer_3at"
#export MR_ROBOT_3_NAME="${MR_HOSTNAME}_11312_${MR_IP_LAST_FIELD}"

export MR_ROBOT_3_NRD="CFixedIntervalsNRD_CM"
export MR_ROBOT_3_ERD="CLoopCloserERD_CM"
export MR_ROBOT_3_GSO="CLevMarqGSO"

#export gazebo_world="simul"
export gazebo_world="ktM"
export num_robots=2 # how many to spawn


# robot coordinates
# http://stackoverflow.com/questions/6659689/referring-to-a-file-relative-to-executing-script
source "${BASH_SOURCE%/*}/scenario_coords/${gazebo_world}.sh"

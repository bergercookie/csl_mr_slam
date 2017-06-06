#!/usr/bin/env bash

# clear the already registered agents
source "${BASH_SOURCE%/*}/unset_registered_agents.sh"

export MR_NUM_OF_ROBOTS=2 # how many to spawn
# state if this is MR-GRAPHSLAM
export MR_IS_MULTIROBOT_GRAPHSLAM=
if (("$MR_NUM_OF_ROBOTS" >= 2)) ; then
    MR_IS_MULTIROBOT_GRAPHSLAM=1
else 
    MR_IS_MULTIROBOT_GRAPHSLAM=0
fi
export MR_IS_MULTIROBOT_GRAPHSLAM

# Computer hostname
export MR_HOSTNAME="$(hostname)"
# Last field of the IP of the computer that runs the simulation
# e.g. 147.102.51.142 => 142
# Used as a suffix in the name of the multimaster configuration


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
###########################################

# use different xacro model based on the ROBOT_X_MODEL variable
# Available models:
# pioneer_3at
# youbot - TODO
# pioneer_3dx - TODO
# pioneer_2dx - TODO

robot_id=1
robot_prefix="MR_ROBOT_${robot_id}"
robot_name="${MR_HOSTNAME}"

export ${robot_prefix}_MODEL="pioneer_3at"

# define the robot name. This will also be the corresponding namespace
export ${robot_prefix}_NAME="${robot_name}"

# deciders optimizers for the current robot agent
export ${robot_prefix}_NRD="CFixedIntervalsNRD_MR"
export ${robot_prefix}_ERD="CLoopCloserERD_MR"
export ${robot_prefix}_GSO="CLevMarqGSO"

# frames of reference

# MR_ROBOT 2
###########################################

if [[ "MR_NUM_OF_ROBOTS" -gt 1 ]]; then
    robot_id=2
    robot_prefix="MR_ROBOT_${robot_id}"
    core_port=11312
    robot_name="${MR_HOSTNAME}_${core_port}"

    export ${robot_prefix}_MODEL="pioneer_3at"
    export ${robot_prefix}_NAME="${robot_name}"
    export ${robot_prefix}_NRD="CFixedIntervalsNRD_MR"
    export ${robot_prefix}_ERD="CLoopCloserERD_MR"
    export ${robot_prefix}_GSO="CLevMarqGSO"

fi # end if MR_NUM_OF_ROBOTS > 1

# MR_ROBOT 3
###########################################

if [[ "MR_NUM_OF_ROBOTS" -gt 2 ]]; then
    robot_id=3
    robot_prefix="MR_ROBOT_${robot_id}"
    core_port=11313
    robot_name="${MR_HOSTNAME}_${core_port}"

    export ${robot_prefix}_MODEL="pioneer_3at"
    export ${robot_prefix}_NAME="${robot_name}"
    export ${robot_prefix}_NRD="CFixedIntervalsNRD_MR"
    export ${robot_prefix}_ERD="CLoopCloserERD_MR"
    export ${robot_prefix}_GSO="CLevMarqGSO"

fi # end if MR_NUM_OF_ROBOTS > 2

# MR_ROBOT 4
###########################################

if [[ "MR_NUM_OF_ROBOTS" -gt 3 ]]; then
    robot_id=4
    robot_prefix="MR_ROBOT_${robot_id}"
    core_port=11314
    robot_name="${MR_HOSTNAME}_${core_port}"

    export ${robot_prefix}_MODEL="pioneer_3at"
    export ${robot_prefix}_NAME="${robot_name}"
    export ${robot_prefix}_NRD="CFixedIntervalsNRD_MR"
    export ${robot_prefix}_ERD="CLoopCloserERD_MR"
    export ${robot_prefix}_GSO="CLevMarqGSO"

fi # end if MR_NUM_OF_ROBOTS > 3

# publish the initial robot position as ROS parameters under the corresponding
# namespace. Mostly for debugging reasons.
# Updte: Not implemented.
export MR_USE_INIT_POSITIONS=0


#export MR_GAZEBO_WORLD="simul"
export MR_GAZEBO_WORLD="ktM"

# Load the robot coordinates
# http://stackoverflow.com/questions/6659689/referring-to-a-file-relative-to-executing-script
source "${BASH_SOURCE%/*}/scenario_coords/${MR_GAZEBO_WORLD}.sh"

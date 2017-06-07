#!/usr/bin/env bash
# Configuration file for running a multi-robot simulation from rosbags

# clear the already registered agents
source "${BASH_SOURCE%/*}/unset_registered_agents.sh"

export MR_NUM_OF_ROBOTS=2 # how many to spawn
export MR_IS_MULTIROBOT_GRAPHSLAM=1

# Computer hostname
export MR_HOSTNAME="$(hostname)"

# Which set of rosbags to use for this demo?
# Available options are
# short
# medium # TODO
# long # TODO
export MR_FILE_SUFFIX="short"

# All nodes read this variable and output the messages accordingly
export MR_OUTPUT_MESSAGES_TO="screen"

# Launch the graphSLAM-related nodes in differeent namespaces
export MR_USE_DIFFERENT_ROSCORES=1

# Syntax for each robot properties
# MR_ROBOT_X_PROPERTYNAME=PROPERTY_VALUE

# MR_ROBOT_1
###########################################

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

# indicates the namespace of the robot as recorded in the rosbag
export ${robot_prefix}_OLD_ROBOT_NS="nickkoukubuntu"

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
    export ${robot_prefix}_OLD_ROBOT_NS="odroidxu3"

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
    export ${robot_prefix}_GSO="odroidu2"

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
    export ${robot_prefix}_GSO="unused"

fi # end if MR_NUM_OF_ROBOTS > 3


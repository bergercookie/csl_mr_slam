#!/usr/bin/env bash

# paths to executables
python_bin=$(which python)
rosbag_bin=$(which rosbag)
rosrun_bin=$(which rosrun)

# load the configuration
source "${BASH_SOURCE%/*}/../config/slam/mr_slam_demo_config.sh"

# configuration parameters
suffix=${MR_FILE_SUFFIX}
bag_pth="$(rospack find mrpt_graphslam_2d)/rosbags/20170607_multi_robot_graphSLAM_${suffix}"
bag_fname="records.bag"
bag_full="${bag_pth}/${bag_fname}"
#bag_full="/home/berger/ktM_upogeio.bag"

# Run the python script - runs the launchfile once for every indicated robot
$python_bin "${BASH_SOURCE%/*}/../nodes/graphslam_demo.py" &

# Get the numeric IDs of the agents that are to be launched.
function get_array_of_robot_ids() {

echo $(printenv | grep -o MR_ROBOT_[0-9]*_NAME | grep -o [0-9])

}

# Return a string containing all the topics that are to be remapped for a single
# agent and have been recorded as part of the combined rosbag
function remap_topics_str() {

old_robot_ns=$1
new_robot_ns=$2

echo " \
/${old_robot_ns}/input/cmd_vel:=/${new_robot_ns}/input/cmd_vel \
/${old_robot_ns}/input/laser_scan:=/${new_robot_ns}/input/laser_scan \
/${old_robot_ns}/input/odom:=/${new_robot_ns}/input/odom \
/${old_robot_ns}/onboard_camera/image_raw:=/${new_robot_ns}/onboard_camera/image_raw \
/${old_robot_ns}/onboard_camera/image_raw/compressed:=/${new_robot_ns}/onboard_camera/image_raw/compressed \
/${old_robot_ns}/onboard_camera/image_raw/compressed/parameter_descriptions:=/${new_robot_ns}/onboard_camera/image_raw/compressed/parameter_descriptions \
/${old_robot_ns}/onboard_camera/image_raw/compressed/parameter_updates:=/${new_robot_ns}/onboard_camera/image_raw/compressed/parameter_updates \
/${old_robot_ns}/onboard_camera/image_raw/theora:=/${new_robot_ns}/onboard_camera/image_raw/theora \
/${old_robot_ns}/onboard_camera/image_raw/theora/parameter_descriptions:=/${new_robot_ns}/onboard_camera/image_raw/theora/parameter_descriptions \
/${old_robot_ns}/onboard_camera/image_raw/theora/parameter_updates:=/${new_robot_ns}/onboard_camera/image_raw/theora/parameter_updates"

}

remap_topics=
for id in $(get_array_of_robot_ids); do
    old_robot_ns_var="MR_ROBOT_${id}_OLD_ROBOT_NS"
    new_robot_ns_var="MR_ROBOT_${id}_NAME"

    printf "Adding remap topics for robot ID: %s\n" $id
    printf "${!old_robot_ns_var} ==> ${!new_robot_ns_var}\n"

    remap_topics="${remap_topics} \
        $(remap_topics_str ${!old_robot_ns_var} ${!new_robot_ns_var})"
done


printf "Topics Remapping:\n\n${remap_topics}\n\n"

$rosbag_bin play --clock ${bag_full} ${remap_topics} \
    output:="${MR_OUTPUT_MESSSAGES_TO}" &

printf "Launching rviz...\n"

rviz_file=$(rospack find mrpt_graphslam_2d)/rviz/graphslam_bag_${MR_NUM_OF_ROBOTS}.rviz
rosrun rviz rviz -d ${rviz_file} &


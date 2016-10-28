Fri Oct 21 19:30:58 EEST 2016, Nikos Koukis

# Description - Purpose

Package holds the scripts for setting up a robot to run as an MR-SLAM agent.
Most of the scripts correspond to the hardware setup and equipment available in
the Control-Systems Lab of the Mechanical Engineering Department of NTUA.

In each one of the MR-SLAM agents (robots) the following shell variables must
be set:
- ROBOT_MODEL  
  String ID of the current agent. Available options are:
  + pioneer_2at
  + pioneer_3at
  + youbot
- ROBOT_ID  
  Numeric ID of the current agent (e.g. 2)
- ROBOT_DRIVERS  
  Available options are:
  + poulias
  + youbot
- WLAN_INTERFACE  
  Wireless interface that is to be used for the communication (e.g. wlan6)
- GEN_ROSBAG_DIR  
  Directory of the generated rosbag

# Usage Information

# See also

% Add link to mrpt_graphslam_2d, graphslam-engine


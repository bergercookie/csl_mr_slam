#!/usr/bin/env bash

# Launch rviz with a delay so that the Robot Models have already been filled in
# the Parameter Server

sleep 2
roslaunch csl_common setup_visuals.launch

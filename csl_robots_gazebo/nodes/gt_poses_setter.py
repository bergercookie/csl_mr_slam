#!/usr/bin/env python
#! Wed Mar 1 20:13:38 EET 2017, Nikos Koukis

import os
import sys
import rospy
import roslaunch
from subprocess import Popen, PIPE, call
from misc.custom_exceptions import RosLaunchSyntaxError
from environ_parser import EnvironParser

class GTPoseSetter(EnvironParser):
    """
    Class that sets the initial Ground-Truth poses in the global coordinates frame.

    Coordinates can be used in a multi-robot setup by other agents so that the
    incoming measurements can be used right away and build the map.

    Warning:
        As of 2017/04/04 current class remains unused. Remove it if not helpful.

    """

    def __init__(self):
        super(GTPoseSetter, self).__init__()


    def setROSParameters(self):
        """Set the robot global position under corresponding topic namespace."""
        for (robot_id, env_params) in self.robot_ID_to_env_params.items():
            rospy.set_param("/{topic_ns}/global_pos".format(
                topic_ns=env_params["name"]), str(env_params["pose_6D"]))

    def _start_launchfile(self, robot_ID):
        super(GTPoseSetter, self)._start_launchfile(robot_ID)





def main():
    """Main function"""
    node_name = "gt_pose_setter"
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} node...".format(node_name))
    rate = rospy.Rate(10.0)

    gt_pose_setter = GTPoseSetter()
    gt_pose_setter.read_env_params()
    gt_pose_setter.setROSParameters()
    # gt_pose_setter.start_launchfiles()
    # rospy.on_shutdown(gt_pose_setter.stop_launchfiles)

    rospy.loginfo("Initialized {} node...".format(node_name))
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()



if __name__ == "__main__":
    main()

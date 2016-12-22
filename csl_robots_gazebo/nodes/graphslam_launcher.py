#!/usr/bin/env python
# Wed Nov 23 20:06:02 EET 2016, Nikos Koukis

import os
import sys
import rospy
import roslaunch
from subprocess import Popen, PIPE, call
from misc.custom_exceptions import RosLaunchSyntaxError
from environ_parser import EnvironParser

class GraphSLAMLauncher(EnvironParser):
    """
    Launch the necessary instances of the mrpt_graphslam_2d node for all the
    robots defined in the shell environment.

    """

    def __init__(self, *arg):
        super(GraphSLAMLauncher, self).__init__()

        # get the launchfile full path
        fname = "run_single_graphSLAM_simul.launch"
        self.launchfile_path = os.path.join(self.csl_gazebo_pkg_path,
                                            "launch", "plumbing",
                                            fname)
        # self.launchfile_errors_to_ignore.append(
            # "Missing package dependencies: csl_robots_gazebo/package.xml: mrpt_graphslam_2d")

        file_ok = self.check_launchfile_for_errors(self.launchfile_path)
        assert(file_ok)

    def _read_env_params(self, robot_ID):
        env_params = super(GraphSLAMLauncher, self)._read_env_params(robot_ID)

        # TODO
        # deciders/optimizers
        for i in ["nrd", "erd", "gso"]:
            env_params[i] = os.environ[self.env_property_prefix + robot_ID + "_" + i.upper()]

        return env_params

    def _start_launchfile(self, robot_ID):
        """Start the run_graphSLAM_simul launchfile for the robot_ID at hand."""

        super(GraphSLAMLauncher, self)._start_launchfile(robot_ID)
        rospy.loginfo("Preparing to execute graphSLAM for robot_ID [{}]...".format(
            robot_ID))

        params = self.robot_ID_to_env_params[robot_ID]

        # Compose the command
        # set the ROS_MASTER_URI environment variable as well - otherwise
        # exception is thrown
        cmd_list = ["export ROS_MASTER_URI=http://localhost:{port}; ".format(port=self.roscore_port)]
        cmd_list.extend([self.cmd])
        cmd_list.extend([self.launchfile_path,
                         "robot_name:={name}".format(name=params["name"])])
        for i in ["nrd", "erd", "gso"]:
            cmd_list.append("{}:={}".format(i.upper(), params[i]))

        cmd_list.append("disable_MRPT_visuals:={}".format("True"))
        cmd_list.extend(self.cmd_port_arg)

        rospy.logwarn("cmd_list - graphslam_launcher = %s", cmd_list)
        return Popen(cmd_list,
                     stdout=PIPE,
                     stderr=PIPE,
                     shell=True)

def main():
    """Main function"""
    node_name = "graphslam_launcher"
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} node...".format(node_name))
    rate = rospy.Rate(10.0)

    graphslam_launcher = GraphSLAMLauncher()
    graphslam_launcher.read_env_params()
    graphslam_launcher.start_launchfiles()

    rospy.on_shutdown(graphslam_launcher.stop_launchfiles)

    rospy.loginfo("Initialized {} node...".format(node_name))
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    main()

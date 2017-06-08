#!/usr/bin/env python
# Wed Jun 7 19:18:05 EEST 2017, Nikos Koukis

import os
import sys
import rospy
import roslaunch
from subprocess import Popen, PIPE, call
from misc.custom_exceptions import RosLaunchSyntaxError
from environ_parser import EnvironParser

class DemoLauncher(EnvironParser):
    """

    Current class provides an automated way of running multi-robot graphSLAM by
    launching the setup_robot_for_mr_demo.launch file as many times as there are
    agents.

    """

    def __init__(self):
        super(DemoLauncher, self).__init__()

        # get the full filepath of the mrpt_graphslam_2d pkg
        mrpt_graphslam_2d_pkg = "mrpt_graphslam_2d"
        tmp = Popen(["rospack", "find", mrpt_graphslam_2d_pkg],
                    stdout=PIPE,
                    stderr=PIPE)
        self.mrpt_graphslam_2d_pkg_path = tmp.communicate()[0].rstrip()

        # get the launchfile full path
        fname = "setup_robot_for_mr_demo.launch"
        self.launchfile_path = os.path.join(self.mrpt_graphslam_2d_pkg_path,
                                            "launch",
                                            fname)

        # Use this in cases when you want to debug using gdb.
        # self.run_under_gdb = False
        self.run_under_gdb = False

        # This raises errors in the launchfile even in cases that the Launchfile
        # is set up correctly
        # file_ok = self.check_launchfile_for_errors(self.launchfile_path)
        file_ok = True
        assert file_ok

        self.read_robot_6D_pose = False;

    def _read_env_params(self, robot_ID):
        env_params = super(DemoLauncher, self)._read_env_params(robot_ID)

        env_params["old_robot_ns"] = os.environ[self.env_property_prefix
                                   + robot_ID +
                                   "_" + "old_robot_ns".upper()]

        # deciders/optimizers
        for i in ["nrd", "erd", "gso"]:
            env_params[i] = os.environ[self.env_property_prefix
                                       + robot_ID +
                                       "_" + i.upper()]

        return env_params

    def _start_launchfile(self, robot_ID):
        """Start the run_graphSLAM_simul launchfile for the robot_ID at hand."""

        super(DemoLauncher, self)._start_launchfile(robot_ID)
        rospy.loginfo("Preparing to execute graphSLAM for robot_ID [{}]...".format(
            robot_ID))

        # parameters for specific robot_ID as set by environment variables
        env_params = self.robot_ID_to_env_params[robot_ID]

        # Compose the command
        # set the ROS_MASTER_URI environment variable as well - otherwise
        # exception is thrown
        cmd_list = []
        cmd_list.extend([self.cmd])

        # robot_name
        cmd_list.extend([self.launchfile_path,
                         "old_robot_ns:={ns}".format(
                             ns=env_params["old_robot_ns"])])
        cmd_list.append("new_robot_ns:={ns}".format(ns=env_params["name"]))

        if self.run_under_gdb:
            # self.run_under_gdb = False
            cmd_list.append("run_under_gdb:=true")

        # NRD, ERD, GSO
        for i in ["nrd", "erd", "gso"]:
            cmd_list.append("{}:={}".format(i.upper(), env_params[i]))

        # launch rviz
        cmd_list.append("start_rviz:={}".format(False))

        # Also set the ROS_MASTER_URI according to the roscore port that is to
        # be used
        env = os.environ
        env["ROS_MASTER_URI"] = "http://localhost:" + self.cmd_port_arg[-1]

        rospy.logwarn("cmd_list - demo_launcher = %s", cmd_list)
        return Popen(cmd_list,
                     # stdout=PIPE,
                     # stderr=PIPE,
                     env=env)

def main():
    """Main function"""
    node_name = "demo_launcher"
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} node...".format(node_name))
    rate = rospy.Rate(10.0)

    demo_launcher = DemoLauncher()
    demo_launcher.read_env_params()
    demo_launcher.start_launchfiles()

    rospy.on_shutdown(demo_launcher.stop_launchfiles)

    rospy.loginfo("Initialized {} node...".format(node_name))
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    main()

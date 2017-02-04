#!/usr/bin/env python
# Tue Dec 13 15:56:04 EET 2016, Nikos Koukis

import abc
import os
import sys
import rospy
import roslaunch
from subprocess import Popen, PIPE, call
from misc.custom_exceptions import RosLaunchSyntaxError

class EnvironParser(object):
    """Provide the necessary methods for parsing graphslam-related environment variables."""
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        # os environment variables
        ######################################################

        # Each robot environment property starts with this
        self.env_mr_prefix = "MR"
        self.env_property_prefix = "{}_ROBOT_".format(self.env_mr_prefix)

        # Make sure that user has set the type of graphSLAM
        self.multirobot_key = "MR_IS_MULTIROBOT_GRAPHSLAM" 
        assert self.multirobot_key in os.environ.keys()

        self.robot_IDs = self.fetch_robot_IDs()

        # port at which the launchfile will be started
        self.init_roscore_port = 11311
        # *current* roscore port. Used only in multimaster setups so that we can
        # simulate multiple robots in the same machine.
        self.roscore_port = self.init_roscore_port

        # Set to True if the launchfiles are to be started in different namespaces
        use_different_roscores_key = self.env_mr_prefix + "_USE_DIFFERENT_ROSCORES"
        assert use_different_roscores_key in os.environ
        self.use_different_roscores = int(os.environ[use_different_roscores_key]) == 1

        self.cmd = "roslaunch"
        self.cmd_port_arg = ["-p", "{port}".format(port=self.roscore_port)]


        # get the full filepath of the csl_gazebo pkg
        csl_gazebo_pkg = "csl_robots_gazebo"
        tmp = Popen(["rospack", "find", csl_gazebo_pkg],
                    stdout=PIPE,
                    stderr=PIPE)
        self.csl_gazebo_pkg_path = tmp.communicate()[0].rstrip()

        # Robot IDs that are to be initialized - Implicitly corresponds to the
        # number of robots that are to be initiialized

        # dict: robot_ID <=> process for stopping the corresponding
        # launchfiles afterwards
        self.robot_ID_to_proc = {}
        self.robot_ID_to_env_params = {}
        self.launchfile_errors_to_ignore = []

    def fetch_robot_IDs(self):
        """Fetch the robot numeric IDs that are to be initialized.

        Find out how many robots have been defined in the shell environment.
        Each robot property is defined in the following way:
        robot_X_property_name

        Use one of the properties, e.g. name (robot_X_name), to find out how
        many robots are to be spawned and their numeric IDs
        """

        robot_ID_keys = [key for key in os.environ.keys() if
                         key.startswith(self.env_property_prefix) and
                         key.endswith("_NAME")]
        robot_IDs = [key.lstrip(self.env_property_prefix).rstrip("_NAME")
                          for key in robot_ID_keys]

        assert len(robot_IDs)
        rospy.logwarn("Found the following robot_IDs: {}".format(robot_IDs))
        return robot_IDs

    def read_env_params(self):
        """Read the environment variables for the robot_IDs available."""

        for robot_ID in self.robot_IDs:
            res = self._read_env_params(robot_ID)
            assert isinstance(res, dict)

            self.robot_ID_to_env_params[robot_ID] = res

    def _read_env_params(self, robot_ID):
        """
        Read the basic robot properties that are defined as shell variables for a
        single robot ID.

        Return: dictionary of defined basic properties
        
        """
        assert isinstance(robot_ID, str)

        # assert that the robot id actually exists

        robot_name_key = self.env_property_prefix + robot_ID + "_NAME"
        assert robot_name_key in os.environ
        robot_type_key = self.env_property_prefix + robot_ID + "_MODEL"
        assert robot_type_key in os.environ

        env_params = {}
        env_params["name"] = os.environ[robot_name_key]
        env_params["model"] = os.environ[robot_type_key]

        # build the 6D Pose (Position + Orientation)
        # take care to use uppercase versions for env variables
        kwords = ["pos", "rot"]
        axes = ["x", "y", "z"]
        pose_prop_combs = ["_".join([kword, axis])
                           for kword in kwords
                           for axis in axes]
        pose_6D = {}
        for pose_prop in pose_prop_combs:
            env_key = self.env_property_prefix + robot_ID + "_" + pose_prop.upper()
            pose_6D[pose_prop] = os.environ[env_key]
        env_params.update({"pose_6D": pose_6D})


        return env_params

    # @staticmethod
    def check_launchfile_for_errors(self, launchfile_path, raise_exc=True):
        success = False

        # check the launchfile syntax first...
        errors = roslaunch.rlutil.check_roslaunch(launchfile_path)
        if (errors is not None and
                errors not in self.launchfile_errors_to_ignore):
            if raise_exc:
                raise RosLaunchSyntaxError(errors)
        else:
            success = True

        return success

    def start_launchfiles(self):
        """
        Start one launchfile for each robot_ID in the
        robot_IDs dict.

        """
        for robot_ID in self.robot_IDs:
            self.cmd_port_arg[-1] = "{port}".format(port=self.roscore_port)
            if self.use_different_roscores:
                self.roscore_port += 1

            res = self._start_launchfile(robot_ID)
            assert isinstance(res, Popen)
            self.robot_ID_to_proc[robot_ID] = res

    @abc.abstractmethod
    def _start_launchfile(self, robot_ID):
        """Start the launchfile for the given robot_ID.

        Return the corresponding Popen object.
        """
        rospy.loginfo("Executing Launchfile for robot_ID: {}".format(robot_ID))

    def stop_launchfiles(self):
        """Stop all the launchfiles that have been launched."""
        pass
    # def stop_launchfiles(self):
        # """ Stop all the launchfiles that have previously been started."""

        # for robot_ID in self.robot_ID_to_proc.keys():
            # self._stop_launchfile(robot_ID)

    # def _stop_launchfile(self, robot_ID):
        # print self.robot_ID_to_env_params
        # rospy.logwarn("Killing launchfile for robot [{}]...".format(
            # self.robot_ID_to_env_params[robot_ID]["name"]))

        # p = self.robot_ID_to_proc[robot_ID]
        # p.kill()
        # # TODO - make sure it's dead

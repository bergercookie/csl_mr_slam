#!/usr/bin/env python
# Wed Nov 23 20:06:02 EET 2016, Nikos Koukis

import os
import sys
import rospy
import roslaunch
from subprocess import Popen, PIPE, call
from misc.custom_exceptions import RosLaunchSyntaxError
from environ_parser import EnvironParser

# Sun Nov 27 20:04:28 EET 2016, Nikos Koukis
# Gazebo is so fucking buggy...
# For the whole operation to be launched correctly follow the following steps:
# - Shutdown the roscore
# - Shutdown Gazebo
# - Launch the setup_simulation_env launchfile
# - If the gzclient doesn't start up launch it manually `$ gzclient`
# - Sometimes the /gazebo/spawn_model service isn't initiated for some reason. Repeat operation again.
# - It takes about ~2m for the robots to be spawned
#
# UPDATE: Gazebo is buggy on a virtual machine without a dedicated graphics card. :-)


class RobotSpawner(EnvironParser):
    """
    Spawn robots in the gazebo world.
    Span the necessary robots based on the environment variables currently
    defined.

    """

    def __init__(self, *arg):
        super(RobotSpawner, self).__init__()

        # self.launchfile_errors_to_ignore.append(
            # "Missing package dependencies: csl_robots_gazebo/package.xml: tf2_ros")

        # get the launchfile full path
        setup_robot_fname = "setup_single_robot.launch"
        self.launchfile_path = os.path.join(self.csl_gazebo_pkg_path,
                                              "launch", "plumbing",
                                              setup_robot_fname)

        file_ok = self.check_launchfile_for_errors(self.launchfile_path)
        assert(file_ok)

        # Dict: robot_name <=> RobotModel instance
        self.robot_model_instances = {}


    def _start_launchfile(self, robot_ID):
        """Method that spawns a robot in the Gazebo world."""

        super(RobotSpawner, self)._start_launchfile(robot_ID)
        robot_model = self.robot_model_instances[robot_ID]
        rospy.loginfo("Preparing to spawn robot [{}]...".format(
            robot_model.robot_name))

        # Compose the command
        cmd = "roslaunch"
        path = self.launchfile_path

        cmd_list = [cmd, path,
                    "robot_name:={}".format(robot_model.robot_name),
                    "robot_type:={}".format(robot_model.robot_type)]
        for k, v in robot_model.pose_6D.items():
            cmd_list.append("{k}:={v}".format(k=k, v=v))

        return Popen(cmd_list, stdout=PIPE)

    def _read_env_params(self, robot_ID):
        """
        Read the robot properties that are defined as shell variables for a
        single robot ID. Based on the latter, initialize a RobotModel instance

        """
        env_params = super(RobotSpawner, self)._read_env_params(robot_ID)

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

        # initialize robot model instance
        robot_model = RobotModel(env_params["name"],
                                 env_params["model"],
                                 pose_6D)
        rospy.loginfo(robot_model)

        env_params.update({"pose_6D": pose_6D})
        self.robot_model_instances[robot_ID] = robot_model

        print env_params
        return env_params


class RobotModel(object):
    """
    Model of a robot that can be spawned in the gazebo world.

    Each robot that is to be spawned in Gazebo must have the following
    properties defined:
    - Robot name: So that the robot namespace can be defined
    - Robot type: Type of the robot type - useful for specifying different
    xacro file for different robots
    - 6D pose at which to spawn

    """

    def __init__(self, robot_name, robot_type, pose_6D):
        super(RobotModel, self).__init__()
        assert type(pose_6D) is dict
        assert len(pose_6D) == 6 or len(pose_6D) == 3

        if len(pose_6D) == 3:
            rot_dict = {"rot_x": 0,
                        "rot_y": 0,
                        "rot_z": 0}
            pose_6D.update(rot_dict)

        self.robot_name = robot_name
        self.robot_type = robot_type
        self.pose_6D = pose_6D

    def __str__(self):
        msg = "Robot model fields:\n"
        msg += "\tRobot Type:\t {}\n".format(self.robot_type)
        msg += "\tRobot Name:\t {}\n".format(self.robot_name)
        for key, prop in self.pose_6D.items():
            msg += "\t {}:\t {}\n".format(key.upper(), prop)

        return msg


def main():
    """Main function"""
    node_name = "robot_spawner"
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} node...".format(node_name))
    rate = rospy.Rate(10.0)

    spawner = RobotSpawner()
    spawner.read_env_params()
    spawner.start_launchfiles()

    rospy.on_shutdown(spawner.stop_launchfiles)

    rospy.loginfo("Initialized {} node...".format(node_name))
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    main()

#!/usr/bin/env python
# Wed Nov 23 20:06:02 EET 2016, Nikos Koukis

import os
import sys
import rospy
import roslaunch
from subprocess import Popen, PIPE, call
from misc.custom_exceptions import RosLaunchSyntaxError

# Sun Nov 27 20:04:28 EET 2016, Nikos Koukis
# Gazebo is so fucking buggy...
# For the whole operation to be launched correctly follow the following steps:
# - Shutdown the roscore
# - Shutdown Gazebo
# - Launch the setup_simulation_env launchfile
# - If the gzclient doesn't start up launch it manually `$ gzclient`
# - Sometimes the /gazebo/spawn_model service isn't initiated for some reason. Repeat operation again.
# - It takes about ~2m for the robots to be spawned


class RobotSpawner(object):
    """
    Spawn robots in the gazebo world.
    Span the necessary robots based on the environment variables currently
    defined.

    """

    def __init__(self, *arg):
        super(RobotSpawner, self).__init__()
        self.arg = arg

        # get the full filepath of the launchfile
        csl_gazebo_pkg = "csl_robots_gazebo"
        tmp = Popen(["rospack", "find", csl_gazebo_pkg], stdout=PIPE)
        csl_gazebo_pkg_path = tmp.communicate()[0].rstrip()
        setup_robot_fname = "setup_single_robot.launch"
        self.setup_robot_launchfile_path = os.path.join(csl_gazebo_pkg_path,
                                                        "launch", "plumbing",
                                                        setup_robot_fname)

        # dict: robot_model <=> process for stopping the corresponding
        # launchfiles afterwards
        self.robot_model_to_process = {}

        # check the launchfile syntax first...
        errors = roslaunch.rlutil.check_roslaunch(
            self.setup_robot_launchfile_path)
        if errors is not None:
            raise RosLaunchSyntaxError(errors)
        assert errors is None

        # Each robot environment property starts with this
        self.env_property_prefix = "MR_ROBOT_"

        # Robot IDs that are to be initialized - Implicitly corresponds to the
        # number of robots that are to be initiialized

        # Dict: robot_name <=> launchfile process
        self.setup_robot_launchfiles = {}
        # Dict: robot_name <=> RobotModel instance
        self.robot_model_instances = {}

        
        self.robot_IDs = []

    def fetch_robot_IDs(self):
        """Fetch the robot numeric IDs that are to be initialized.

        Find out how many robots have been defined in the shell environment.
        Each robot property is defined in the following way:
        robot_X_property_name

        Use one of the properties, e.g. name (robot_X_name), to find out how
        many robots are to be spawned and their numeric IDs
        """

        # Make sure that user has set the type of graphSLAM
        multirobot_key = "MR_IS_MULTIROBOT_GRAPHSLAM" 
        assert multirobot_key in os.environ.keys()

        if (int(os.environ[multirobot_key]) is 1):

            robot_ID_keys = [key for key in os.environ.keys() if
                             key.startswith(self.env_property_prefix) and
                             key.endswith("_NAME")]
            self.robot_IDs = [key.lstrip(self.env_property_prefix).rstrip("_NAME")
                              for key in robot_ID_keys]
        else:
            # we have only one robot - and that must have ID = 
            self.robot_IDs = 1

        assert len(self.robot_IDs)
        rospy.logwarn("Found the following robot_IDs: {}".format(self.robot_IDs))

    def start_launchfiles(self):
        """
        Start one setup_single_robot launchfile for each robot_ID in the
        robot_IDs dict.

        """
        for robot_ID, robot_model in self.robot_model_instances.items():
            self._start_setup_single_robot_launchfile(robot_model)

    def _start_setup_single_robot_launchfile(self, robot_model):
        """Method that spawns a robot in the Gazebo world."""

        rospy.loginfo("Preparing to spawn robot_ID [{}]".format(
            robot_model.robot_name))

        cmd = "roslaunch"
        path = self.setup_robot_launchfile_path

        # Position args
        pos_args_str = "".join(["-x {pos_x} -y {pos_y} -z {pos_z} ",
                                "-R {rot_x} -P {rot_y} -Y {rot_z}"]).format(
                                    pos_x=robot_model.pose_6D["pos_x"],
                                    pos_y=robot_model.pose_6D["pos_y"],
                                    pos_z=robot_model.pose_6D["pos_z"],
                                    rot_x=robot_model.pose_6D["rot_x"],
                                    rot_y=robot_model.pose_6D["rot_y"],
                                    rot_z=robot_model.pose_6D["rot_z"])

        self.robot_model_to_process[robot_model] = Popen(
            [cmd, path,
             "robot_name:={}".format(robot_model.robot_name),
             "robot_type:={}".format(robot_model.robot_type),
             "position_args:={}".format(pos_args_str)],
            stdout=PIPE)

        pass

    def stop_launchfiles(self):
        """
        Stop all the launchfiles that have previously been started.

        """

        for robot_model in self.robot_model_to_process.keys():
            self._stop_setup_single_robot(robot_model)

    def _stop_setup_single_robot(self, robot_model):
        """Stop the robot model that has the robot_name provided."""
        rospy.logwarn("Killing launchfile for robot [{}]".format(
            robot_model.robot_name))

        p = self.robot_model_to_process[robot_model]
        p.kill()

    def read_env_params(self):
        """Read the environment variables for the robot_IDs available."""

        for robot_ID in self.robot_IDs:
            self.read_robot_env_params(robot_ID)

    def read_robot_env_params(self, robot_ID):
        """
        Read the robot properties that are defined as shell variables based for
        a single robot ID. Based on the latter, initialize a RobotModel
        instance
        
        """
        # assert that the robot id actually exists
        robot_name_property = "".join([self.env_property_prefix,
                                       str(robot_ID),
                                       "_NAME"])
        assert robot_name_property in os.environ

        # build the 6D Pose (Position + Orientation)
        prefix = self.env_property_prefix

        # take care to use uppercase versions for env variables
        kwords = ["pos", "rot"]
        axes = ["x", "y", "z"]
        pose_prop_combs = ["_".join([kword, axis])
                           for kword in kwords
                           for axis in axes]

        pose_6D = {}
        for pose_prop in pose_prop_combs:
            env_key = prefix + robot_ID + "_" + pose_prop.upper()
            pose_6D[pose_prop] = os.environ[env_key]

        robot_name_field = prefix + robot_ID + "_NAME"
        robot_type_field = prefix + robot_ID + "_MODEL"
        robot_name = os.environ[robot_name_field]
        robot_type = os.environ[robot_type_field]

        # initialize robot model instance - start corresponding launchfile
        robot_model = RobotModel(robot_name, robot_type, pose_6D)
        rospy.loginfo(robot_model)

        self.robot_model_instances[str(robot_ID)] = robot_model


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
    spawner.fetch_robot_IDs()
    spawner.read_env_params()
    spawner.start_launchfiles()

    rospy.on_shutdown(spawner.stop_launchfiles)

    rospy.loginfo("Initialized {} node...".format(node_name))
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    main()

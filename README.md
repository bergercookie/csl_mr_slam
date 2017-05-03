# csl_mr_slam

Repository consists of the following ROS catkin packages:

- csl_hw_setup:

    Contains shell scripts, launchfiles as well as miscellaneous configuration
    files for setting up and launching single-and multi robot graphSLAM. Overall
    package is designed based on the hardware available in the Control Systems
    Lab of the Mechanical Engineering Department of NTUA.

- csl_robots_gazebo:

    Contains shell scripts, launchfiles for setting up a graphSLAM simulation in
    the gazebo environment.

- csl_robot_descriptions:

    Helper package containing robot description that have to be available for
    simulating graphSLAM in Gazebo. These include xacro, sdf files for the used
    robots (e.g. Pioneer), the onboard cameras and the laser rangefinders.

- csl_common:

    Holds configuration common to both the simulation as well as the real-time experiments


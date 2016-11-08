# Plumbing directory documentation

Wed Oct 26 08:49:09 EEST 2016, Nikos Koukis

Plumbing (as in git-plumbing commands) roslaunch files are stored here.
There should not be directly be executed by the user but they are to be
included in higher level launchfiles found under the porcelain/ directory.

Typical operations of files found in this directory include:

- Setup of hardware sensors, robot drivers etc.
- Cameras setup
- Setup of nodes that handle the robot teleoperation

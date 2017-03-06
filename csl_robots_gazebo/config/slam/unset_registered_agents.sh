# Mon Mar 6 12:06:27 EET 2017, Nikos Koukis
# source this file in all working bash sessions to unset the registered agents.
# graphslam_launcher checks whether the MR_ROBOT_X_NAME variables exist, not
# whether they have a valid value or not
#
# Script is automatically called from the simulation_config.sh

max_agent_num=100
printf "Unsetting the first %d agent names (1 -> 100)\n" $max_agent_num
for i in $(seq 1 $max_agent_num); do
    unset MR_ROBOT_"$i"_NAME
done

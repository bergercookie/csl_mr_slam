# robot1
export MR_ROBOT_1_POS_X=-2.76
export MR_ROBOT_1_POS_Y=3
export MR_ROBOT_1_POS_Z=0.051
export MR_ROBOT_1_ROT_X=0
export MR_ROBOT_1_ROT_Y=0
export MR_ROBOT_1_ROT_Z=0

# robot2
if [[ "num_robots" -gt 1 ]]; then

    export MR_ROBOT_2_POS_X=-1.83
    export MR_ROBOT_2_POS_Y=-5
    export MR_ROBOT_2_POS_Z=0
    export MR_ROBOT_2_ROT_X=0
    export MR_ROBOT_2_ROT_Y=0
    export MR_ROBOT_2_ROT_Z=0

fi

# robot3
if [[ "num_robots" -gt 2 ]]; then

    export MR_ROBOT_3_POS_X=8.74
    export MR_ROBOT_3_POS_Y=3.0
    export MR_ROBOT_3_POS_Z=0.051
    export MR_ROBOT_3_ROT_X=0
    export MR_ROBOT_3_ROT_Y=0
    export MR_ROBOT_3_ROT_Z=3.14

fi

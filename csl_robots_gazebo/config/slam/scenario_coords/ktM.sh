# ktM
# standard

export MR_ROBOT_1_POS_X=-1.5
export MR_ROBOT_1_POS_Y=-3
export MR_ROBOT_1_POS_Z=0.051
export MR_ROBOT_1_ROT_X=0
export MR_ROBOT_1_ROT_Y=0
export MR_ROBOT_1_ROT_Z=0

if [[ "MR_NUM_OF_ROBOTS" -gt 1 ]]; then

    export MR_ROBOT_2_POS_X=10
    export MR_ROBOT_2_POS_Y=-6
    export MR_ROBOT_2_POS_Z=0.051
    export MR_ROBOT_2_ROT_X=0
    export MR_ROBOT_2_ROT_Y=0
    export MR_ROBOT_2_ROT_Z=3.1415


fi # end if MR_NUM_OF_ROBOTS > 1

# robot 3

if [[ "MR_NUM_OF_ROBOTS" -gt 2 ]]; then

    export MR_ROBOT_3_POS_X=-1.5
    export MR_ROBOT_3_POS_Y=-5.2
    export MR_ROBOT_3_POS_Z=0.051
    export MR_ROBOT_3_ROT_X=0
    export MR_ROBOT_3_ROT_Y=0
    export MR_ROBOT_3_ROT_Z=0

fi # MR_NUM_OF_ROBOTS > 2

if [[ "MR_NUM_OF_ROBOTS" -gt 3 ]]; then

    # robot 4
    export MR_ROBOT_4_POS_X=2.5
    export MR_ROBOT_4_POS_Y=-6
    export MR_ROBOT_4_POS_Z=0.051
    export MR_ROBOT_4_ROT_X=0
    export MR_ROBOT_4_ROT_Y=0
    export MR_ROBOT_4_ROT_Z=3.1416

fi # MR_NUM_OF_ROBOTS > 3

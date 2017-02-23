# 0 -> same room - opposite direction
# 1 -> same room - same direction
# 2 -> other room - opposite direction
robot2_pos=2;

# ktM
# standard

export MR_ROBOT_1_POS_X=-1.5
export MR_ROBOT_1_POS_Y=-3
export MR_ROBOT_1_POS_Z=0.051
export MR_ROBOT_1_ROT_X=0
export MR_ROBOT_1_ROT_Y=0
export MR_ROBOT_1_ROT_Z=0

if [[ "num_robots" -gt 1 ]]; then

    # robot 2
    if [[ "$robot2_pos" == 0 ]]; then

        export MR_ROBOT_2_POS_X=2.5
        export MR_ROBOT_2_POS_Y=-6
        export MR_ROBOT_2_POS_Z=0.051
        export MR_ROBOT_2_ROT_X=0
        export MR_ROBOT_2_ROT_Y=0
        export MR_ROBOT_2_ROT_Z=3.1416

    elif [[ "$robot2_pos" == 1 ]]; then
        export MR_ROBOT_2_POS_X=-1.5
        export MR_ROBOT_2_POS_Y=-5.2
        export MR_ROBOT_2_POS_Z=0.051
        export MR_ROBOT_2_ROT_X=0
        export MR_ROBOT_2_ROT_Y=0
        export MR_ROBOT_2_ROT_Z=0

    elif [[ "$robot2_pos" == 2 ]]; then
        export MR_ROBOT_2_POS_X=10
        export MR_ROBOT_2_POS_Y=-6
        export MR_ROBOT_2_POS_Z=0.051
        export MR_ROBOT_2_ROT_X=0
        export MR_ROBOT_2_ROT_Y=0
        export MR_ROBOT_2_ROT_Z=3.1415


    fi # end if robot2_pos

fi # end if num_robots > 1

# robot 3

if [[ "num_robots" -gt 2 ]]; then

    export MR_ROBOT_3_POS_X=-1.5
    export MR_ROBOT_3_POS_Y=-5.2
    export MR_ROBOT_3_POS_Z=0.051
    export MR_ROBOT_3_ROT_X=0
    export MR_ROBOT_3_ROT_Y=0
    export MR_ROBOT_3_ROT_Z=0

fi # num_robots > 2



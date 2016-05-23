
    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Database updates after resets

    printf "\nChoose what you want to change in the Database\n"
    printf " 1. Turn off mock settings,\n 2. Set tack_min_speed in course_calculation_config(1),\n 3. Set loop_time in sailing_robot_config(0.1),\n \033[32m4. Do all of the above,\033[0m\n 5. Manually change waypoint routing,\n 6. Set database server settings\n"
    read -p " Choice: " C

    if [ $C -eq 1 ]
    then
            sqlite3 asr.db "UPDATE mock SET gps="0", windsensor="0", analog_arduino="1", compass="0", position="0", maestro="0";"
            printf "\n\033[32m mock updated.\033[0m\n\n"

    elif [ $C -eq 2 ]
    then
            sqlite3 asr.db "UPDATE course_calculation_config SET tack_min_speed="1";"
            printf "\n\033[32m course_calculation_config updated.\033[0m\n\n"
    elif [ $C -eq 3 ]
    then
            sqlite3 asr.db "UPDATE sailing_robot_config SET loop_time="0.1";"
            printf "\n\033[32m sailing_robot_config updated.\033[0m\n\n"
    elif [ $C -eq 4 ]
    then
            sqlite3 asr.db "UPDATE mock SET gps="0", windsensor="0", analog_arduino="1", compass="0", position="0", maestro="0";"
            printf "\n\033[36m mock updated.\033[0m\n"

            sqlite3 asr.db "UPDATE course_calculation_config SET tack_min_speed="1";"
            printf "\033[36m course_calculation_config updated.\033[0m\n"

            sqlite3 asr.db "UPDATE sailing_robot_config SET loop_time="0.1";"
            printf "\033[36m sailing_robot_config updated.\033[0m\n\n"

    elif [ $C -eq 5 ]
    then
            printf "\n\033[36m Choose table to change in waypoint_routing_config\033[0m\n"
            printf " 1. sail_adjust_time,\n 2. adjust_degree_limit,\n 3. rudder_speed_min,\n"
            read -p " Choice: " C

            if [ $C -eq 1 ]
            then
                printf "\nSet sail_adjust_time(default=0.5): "
                read -p " Choice: " SAT
                sqlite3 asr.db "UPDATE waypoint_routing_config SET sail_adjust_time="$SAT";"
                printf "\n \033[32m sail_adjust_time SET to $SAT\033[0m\n\n"

            elif [ $C -eq 2 ]
            then
                printf "\nSet adjust_degree_limit(default=10): "
                read -p " Choice: " ADL
                sqlite3 asr.db "UPDATE waypoint_routing_config SET adjust_degree_limit="$ADL";"
                printf "\n \033[32m adjust_degree_limit SET to $ADL\033[0m\n\n"

            elif [ $C -eq 3 ]
            then
                printf "\nSet rudder_speed_min(default=1.0): "
                read -p " Choice: " RSM
                sqlite3 asr.db "UPDATE waypoint_routing_config SET rudder_speed_min="$RSM";"
                printf "\n \033[32m rudder_speed_min SET to $RSM\033[0m\n\n"
            fi

    elif [ $C -eq 6 ]
    then

        sqlite3 asr.db "UPDATE server SET boat_id='BOAT02';"
        printf "\n \033[32m Server id set to BOAT02\033[0m"
        sqlite3 asr.db "UPDATE server SET boat_pwd='PWD02';"
        printf "\n \033[32m Server password set to PWD02\033[0m"
        sqlite3 asr.db "UPDATE server SET srv_addr='http://www.sailingrobots.com/testdata/sync/';"
        printf "\n \033[32m Server address set to http://www.sailingrobots.com/testdata/sync/\033[0m\n\n"

fi

# chmod +x updateDB.sh

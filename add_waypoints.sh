  
    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Add waypoints

    printf "This will insert waypoints in the database\n"
    read -p "Enter number of waypoints you wish to enter:" COUNT

    COUNTER=1
    C=0
    while [  $C -lt $COUNT ]; do
        printf "\nWaypoint $COUNTER/$COUNT\n"
        read -p " enter LAT:" LAT
        read -p " enter LON:" LON
        echo sqlite3 asr.db "insert into waypoints (id,lat,lon) values ('$COUNT',$LAT,'$LON');"
        let COUNTER=COUNTER+1
        let C=C+1 
    done

	printf "Done\n"
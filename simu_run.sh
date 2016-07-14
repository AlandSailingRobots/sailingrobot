#!/bin/bash

SIMU_PATH=/home/sailbot/sailing_simulator/PROGSIDE

sh -c "$SIMU_PATH/socket_to_sr $SIMU_PATH/port_path.sh" &
sleep 1
source $SIMU_PATH/port_path.sh

echo $CV7_PORT_SIMU
echo $GPS_PORT_SIMU
echo $MAESTRO_PORT_SIMU

gpsd -N $GPS_PORT_SIMU &

sqlite3 simu_asr.db "UPDATE maestro_controller_config SET port='$MAESTRO_PORT_SIMU';"
sqlite3 simu_asr.db "UPDATE windsensor_config SET port='$CV7_PORT_SIMU';"

#python $SIMU_PATH/../SIMSIDE/python/simulation_main.py &

sleep 1

LD_PRELOAD=$SIMU_PATH/libwiringPiH.so ./sr ./ simu_asr.db

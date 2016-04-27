#!/bin/bash

    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Add waypoints

printf "\nThis will insert waypoints in the database\n"
printf "1. ÅSS route, 2. Svibybron route, 3. Short route\n"
read -p "Choice: " C

if [ $C -eq 1 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107429, 19.923938, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.107101, 19.922842, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.104441, 19.921654, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.101783, 19.921775, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.101783, 19.921775, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.099907, 19.921147, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7, 60.099864, 19.920375, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (8, 60.100169, 19.919871, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (9, 60.101854, 19.920922, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (10, 60.107099, 19.922323, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (11, 60.107377, 19.924115, 6, 15, 0);"

elif [ $C -eq 2 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"
      
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107429, 19.923938, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.108772, 19.922876, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.113281, 19.922032, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.113261, 19.920371, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.108502, 19.922886, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.107429, 19.923938, 6, 15, 0);"
elif [ $C -eq 3 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"
      
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107429, 19.923938, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.107572, 19.920289, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.109529, 19.921373, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.109299, 19.924892, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.107978, 19.922199, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.107429, 19.923938, 6, 15, 0);"

fi



printf "Done\n"



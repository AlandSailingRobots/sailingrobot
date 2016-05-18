#!/bin/bash

    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Add waypoints

printf "\nThis will insert waypoints in the database\n"
printf "1. ÅSS route, 2. Short route, 3. Long Harbour route\n"
read -p "Choice: " C

if [ $C -eq 1 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107101, 19.922842, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.104441, 19.921654, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.101783, 19.921775, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.101783, 19.921775, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.099907, 19.921147, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.099864, 19.920375, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7, 60.100169, 19.919871, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (8, 60.101854, 19.920922, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (9, 60.107099, 19.922323, 6, 15, 0);"

elif [ $C -eq 2 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107415, 19.922481, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.106236, 19.922452, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.104709, 19.922034, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.104634, 19.920027, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.107097, 19.920955, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.107415, 19.922481, 6, 15, 0);"
elif [ $C -eq 3 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107464, 19.922077, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.076216, 19.923036, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.107342, 19.921907, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.103847, 19.920787, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.079803, 19.922063, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.101038, 19.920719, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7, 60.078905, 19.917712, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (8, 60.099931, 19.920847, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (9, 60.077419, 19.922884, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (10, 60.098854, 19.921091, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (11, 60.085830, 19.926167, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (12, 60.097491, 19.921879, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (13, 60.086950, 19.925755, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (14, 60.094855, 19.921304, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (15, 60.091133, 19.918932, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (16, 60.089916, 19.920983, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (17, 60.094149, 19.921904, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (18, 60.086341, 19.926303, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (19, 60.097382, 19.922033, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (20, 60.082129, 19.924239, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (21, 60.098451, 19.920969, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (22, 60.078823, 19.917433, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (23, 60.100148, 19.920535, 6, 15, 0);"

fi



printf "Done\n"

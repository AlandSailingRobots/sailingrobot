#!/bin/bash

    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Add waypoints

printf "\nThis will insert waypoints in the database\n"
printf "1. ÅSS route, 2. Short route, 3. Long Harbour route, 4. Circle around ÅSS, 5. EckeroToGrisslehamn,\n\t 6. Test Cable North to South 7. Test Cable South to North\n"
read -p "Choice: " C

if [ $C -eq 1 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107101, 19.922842, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.104441, 19.921654, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.101783, 19.921775, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.099907, 19.921147, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.099864, 19.920375, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.100169, 19.919871, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7, 60.101854, 19.920922, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (8, 60.107099, 19.922323, 6, 15, 0);"

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
elif [ $C -eq 4 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.107101, 19.922842, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.104441, 19.921654, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.101783, 19.921775, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.100094, 19.921539, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.099065, 19.921859, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.097813, 19.922358, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7, 60.097802, 19.921231, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (8, 60.100058, 19.920913, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (9, 60.100010, 19.921713, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (10, 60.097735, 19.922570, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (11, 60.097666, 19.921369, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (12, 60.100011, 19.920900, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (13, 60.100070, 19.921582, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (14, 60.098990, 19.921823, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (15, 60.097723, 19.922552, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (16, 60.097707, 19.921200, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (17, 60.100151, 19.920995, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (18, 60.102203, 19.920921, 6, 5, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (19, 60.107353, 19.922158, 6, 5, 0);"
elif [ $C -eq 5 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1, 60.22654833, 19.53291333, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2, 60.21526667, 19.51169, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3, 60.20231833, 19.487045, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4, 60.1851725, 19.471833335, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5, 60.17202667, 19.43062167, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6, 60.15221, 19.34759667, 6, 50, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7, 60.1378970825, 19.22525167, 6, 100, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (8, 60.123584165, 60.123584165, 6, 100, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (9, 60.1092712475, 18.98056167, 6, 100, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (10, 60.09495833, 18.85821667, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (11, 60.09387667, 18.846075, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (12, 60.09588667, 18.82695833, 6, 10, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (13, 60.09882167, 18.813265, 6, 10, 0);"
elif [ $C -eq 6 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1,  60.102500,  19.920000, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2,  60.101675,  19.920471, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3,  60.100705,  19.920993, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4,  60.099852,  19.921348, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5,  60.100705,  19.920993, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6,  60.101675,  19.920471, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7,  60.102500,  19.920000, 6, 15, 0);"
elif [ $C -eq 7 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (1,  60.099852,  19.921348, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (2,  60.100705,  19.920993, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (3,  60.101675,  19.920471, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (4,  60.102500,  19.920000, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (5,  60.101675,  19.920471, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (6,  60.100705,  19.920993, 6, 15, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,harvested) VALUES (7,  60.099852,  19.921348, 6, 15, 0);"
fi



printf "Done\n"

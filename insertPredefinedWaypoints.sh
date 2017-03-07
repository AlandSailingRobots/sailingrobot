#!/bin/bash

    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Add waypoints

printf "\nThis will insert waypoints in the database\n"
printf "1. ÅSS route, 2. Short route, 3. Long Harbour route, 4. Circle around ÅSS, 5. EckeroToGrisslehamn,\n\t 6. Test Cable North to South 7. Test Cable South to North 8. Test Left of Harbour (left Sviby) 9. Triangle 10. Tack Test 11. Long Tack Test 12. Long Tour\n" 
read -p "Choice: " C

if [ $C -eq 1 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1, 60.107240, 19.922397, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2, 60.105700, 19.922311, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3, 60.103818, 19.921925, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4, 60.101492, 19.921420, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5, 60.101508, 19.920401, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6, 60.103861, 19.920659, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7, 60.105775, 19.920895, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (8, 60.107336, 19.921474, 6, 20, 0, 0);"

elif [ $C -eq 2 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1, 60.107415, 19.922481, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2, 60.106236, 19.922452, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3, 60.104709, 19.922034, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4, 60.104634, 19.920027, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5, 60.107097, 19.920955, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6, 60.107415, 19.922481, 6, 15, 0, 0);"
elif [ $C -eq 3 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1, 60.107464, 19.922077, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2, 60.076216, 19.923036, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3, 60.107342, 19.921907, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4, 60.103847, 19.920787, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5, 60.079803, 19.922063, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6, 60.101038, 19.920719, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7, 60.078905, 19.917712, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (8, 60.099931, 19.920847, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (9, 60.077419, 19.922884, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (10, 60.098854, 19.921091, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (11, 60.085830, 19.926167, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (12, 60.097491, 19.921879, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (13, 60.086950, 19.925755, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (14, 60.094855, 19.921304, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (15, 60.091133, 19.918932, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (16, 60.089916, 19.920983, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (17, 60.094149, 19.921904, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (18, 60.086341, 19.926303, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (19, 60.097382, 19.922033, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (20, 60.082129, 19.924239, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (21, 60.098451, 19.920969, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (22, 60.078823, 19.917433, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (23, 60.100148, 19.920535, 6, 15, 0, 0);"
elif [ $C -eq 4 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"

    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1, 60.107101, 19.922842, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2, 60.104441, 19.921654, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3, 60.101783, 19.921775, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4, 60.100094, 19.921539, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5, 60.099065, 19.921859, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6, 60.097813, 19.922358, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7, 60.097802, 19.921231, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (8, 60.100058, 19.920913, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (9, 60.100010, 19.921713, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (10, 60.097735, 19.922570, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (11, 60.097666, 19.921369, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (12, 60.100011, 19.920900, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (13, 60.100070, 19.921582, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (14, 60.098990, 19.921823, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (15, 60.097723, 19.922552, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (16, 60.097707, 19.921200, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (17, 60.100151, 19.920995, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (18, 60.102203, 19.920921, 6, 5, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (19, 60.107353, 19.922158, 6, 5, 0, 0);"
elif [ $C -eq 5 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1, 60.22654833, 19.53291333, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2, 60.21526667, 19.51169, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3, 60.20231833, 19.487045, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4, 60.1851725, 19.471833335, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5, 60.17202667, 19.43062167, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6, 60.15221, 19.34759667, 6, 50, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7, 60.1378970825, 19.22525167, 6, 100, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (8, 60.123584165, 60.123584165, 6, 100, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (9, 60.1092712475, 18.98056167, 6, 100, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (10, 60.09495833, 18.85821667, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (11, 60.09387667, 18.846075, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (12, 60.09588667, 18.82695833, 6, 10, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (13, 60.09882167, 18.813265, 6, 10, 0, 0);"
elif [ $C -eq 6 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.102500,  19.920000, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.101675,  19.920471, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3,  60.100705,  19.920993, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4,  60.099852,  19.921348, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5,  60.100705,  19.920993, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6,  60.101675,  19.920471, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7,  60.102500,  19.920000, 6, 15, 0, 0);"
elif [ $C -eq 7 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.099852,  19.921348, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.100705,  19.920993, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3,  60.101675,  19.920471, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4,  60.102500,  19.920000, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5,  60.101675,  19.920471, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6,  60.100705,  19.920993, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7,  60.099852,  19.921348, 6, 15, 0, 0);"
elif [ $C -eq 8 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.07377046212067,  19.89842141866977, 6, 20, 60, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.07440308841898,  19.89854461396451, 6, 15, 40, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3,  60.07476604297514,  19.89860698629756, 6, 10, 10, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4,  60.07508892948525,  19.89867817656525, 6, 10, 20, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5,  60.07485276924425,  19.8974246147239, 6, 10, 10, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6,  60.07466922062591,  19.89640962820783, 6, 10, 10, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7,  60.07442092694945,  19.89496570322077, 6, 10, 10, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (8,  60.07415298302343,  19.89604995407627, 6, 10, 10, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (9,  60.07394283093122,  19.89719099349307, 6, 10, 10, 0);"
elif [ $C -eq 9 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.103605,  19.921601, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.106473,  19.923030, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3,  60.106600,  19.920753, 6, 10, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4,  60.103605,  19.921601, 6, 20, 0, 0);"
elif [ $C -eq 10 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.103605,  19.921601, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.101808,  19.921478, 6, 20, 0, 0);"
elif [ $C -eq 11 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.103605,  19.921601, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.096779,  19.921943, 6, 40, 0, 0);"
elif [ $C -eq 12 ]
then
    sqlite3 asr.db "DROP TABLE waypoints;"
    sqlite3 asr.db "create table waypoints ( id PRIMARY KEY, latitude INTEGER, longitude INTEGER, declination INTEGER, radius INTEGER, stay_time INTEGER, harvested BOOLEAN );"
    
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (1,  60.1074514,  19.9236470, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (2,  60.1075770,  19.9216461, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (3,  60.1040933,  19.9208951, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (4,  60.1006921,  19.9207449, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (5,  60.0974403,  19.9212813, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (6,  60.0955254,  19.9217105, 6, 20, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (7,  60.0925192,  19.9205518, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (8,  60.0904436,  19.9204659, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (9,  60.0917489,  19.9234486, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (10,  60.0969054,  19.9228048, 6, 40, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (11,  60.1006921,  19.9211955, 6, 15, 0, 0);"
    sqlite3 asr.db "INSERT INTO waypoints (id,latitude,longitude,declination,radius,stay_time,harvested) VALUES (12,  60.1076011,  19.9222040, 6, 40, 0, 0);"
fi


printf "Done\n"

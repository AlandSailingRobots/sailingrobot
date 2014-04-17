#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailingrobot
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -Wall -pedantic -Werror
LIBS = -lsqlite3 -lrt -lwiringPi -lgps

SRC1 = SailingRobot.cpp example.cpp ../coursecalculation/CourseCalculation.cpp ../dbhandler/DBHandler.cpp ../ruddercommand/RudderCommand.cpp ../sailcommand/SailCommand.cpp ../waypointlist/WaypointList.cpp
SRC2 = ../servocontroller/ServoObject.cpp ../servocontroller/MaestroController.cpp ../windsensor/WindSensorController.cpp ../windsensor/AdapterWaleswind.cpp ../windsensor/AdapterCV7.cpp ../gps/GPSReader.cpp
SOURCES = $(SRC1) $(SRC2)
HEADERS =
FILE = sr



all : $(FILE)

$(FILE) : $(SOURCES)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -o $(FILE)

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
LIBS = -lsqlite3 

SRC1 = SailingRobot.cpp example.cpp ../coursecalculation/CourseCalculation.cpp ../dbhandler/DBHandler.cpp ../ruddercommand/RudderCommand.cpp ../sailcommand/SailCommand.cpp ../waypointlist/WaypointList.cpp
SRC2 = ../servocontroller/ServoObject.cpp ../servocontroller/MaestroController.cpp ../windsensor/WindSensorController.cpp ../windsensor/AdapterWaleswind.cpp
SOURCES = $(SRC1) $(SRC2)
HEADERS =
FILE = sr



all : $(FILE)

$(FILE) : $(SOURCES)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -o $(FILE)

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
LIBS =

OBJS = ./coursecalculation/CourseCalculation.o ../dbhandler/DBHandler.o ../ruddercommand/RudderCommand.o ../sailcommand/SailCommand.o ../waypointlist/WaypointList.o ../servocontroller/ServoObject.o ../servocontroller/MaestroController.o ./windsensor/WindSensorController.o ../windsensor/AdapterWaleswind.o ../windsensor/AdapterCV7.o ../gps/GPSReader.o
SRC1 = SailingRobot.cpp example.cpp ../coursecalculation/CourseCalculation.cpp ../dbhandler/DBHandler.cpp ../ruddercommand/RudderCommand.cpp ../sailcommand/SailCommand.cpp ../waypointlist/WaypointList.cpp
SRC2 = ../servocontroller/ServoObject.cpp ../servocontroller/MaestroController.cpp ../windsensor/WindSensorController.cpp ../windsensor/AdapterWaleswind.cpp ../windsensor/AdapterCV7.cpp ../gps/GPSReader.cpp
SOURCES = $(SRC1) $(SRC2)
HEADERS =
FILE = sr



all : $(FILE)

$(FILE) : $(SOURCES)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -o $(FILE)

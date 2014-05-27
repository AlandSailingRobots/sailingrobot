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
LIBS = -lsqlite3 -lgps -lrt -lwiringPi

OBJS1 = ../coursecalculation/CourseCalculation.o ../dbhandler/DBHandler.o ../ruddercommand/RudderCommand.o ../sailcommand/SailCommand.o ../waypointlist/WaypointList.o ../servocontroller/ServoObject.o 
OBJS2 = ../servocontroller/MaestroController.o ../windsensor/WindSensorController.o ../windsensor/AdapterWaleswind.o ../windsensor/AdapterCV7.o ../gps/GPSReader.o
OBJS3 = ../servocontroller/SensorObject.o
OBJECTS = $(OBJS1) $(OBJS2) $(OBJS3)
SOURCES = SailingRobot.cpp example.cpp
HEADERS = SailingRobot.h
FILE = sr



all : coursecalculation dbhandler ruddercommand sailcommand waypointlist servocontroller windsensor gps $(FILE)

coursecalculation :
	cd ../coursecalculation && $(MAKE)

dbhandler :
	cd ../dbhandler && $(MAKE)

ruddercommand :
	cd ../ruddercommand && $(MAKE)

sailcommand :
	cd ../sailcommand && $(MAKE)

waypointlist :
	cd ../waypointlist && $(MAKE)

servocontroller :
	cd ../servocontroller && $(MAKE)

windsensor :
	cd ../windsensor && $(MAKE)

gps :
	cd ../gps && $(MAKE)

$(FILE) : $(SOURCES) $(HEADERS) $(OBJECTS)
	$(CC) $(SOURCES) $(OBJECTS) $(FLAGS) $(LIBS) -o $(FILE)

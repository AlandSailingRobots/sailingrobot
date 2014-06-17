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

COURSE = coursecalculation/CourseCalculation.o 
DB = dbhandler/DBHandler.o 
COMMAND = ruddercommand/RudderCommand.o sailcommand/SailCommand.o 
WAYPOINT = waypointlist/WaypointList.o 
MAESTRO = servocontroller/MaestroController.o servocontroller/ServoObject.o servocontroller/SensorObject.o 
CV7 = windsensor/WindSensorController.o windsensor/AdapterWaleswind.o windsensor/AdapterCV7.o
GPS = gps/GPSReader.o gps/MockGPSReader.o
HTTP = httpsync/HTTPSync.o httpsync/JSON.o

OBJECTS = $(COURSE) $(DB) $(COMMAND) $(WAYPOINT) $(MAESTRO) $(CV7) $(GPS) $(HTTP)
SOURCES = SailingRobot.cpp example.cpp
HEADERS = SailingRobot.h
FILE = sr



all : coursecalculation dbhandler ruddercommand sailcommand waypointlist servocontroller windsensor gps httpsync $(FILE)

coursecalculation :
	cd coursecalculation && $(MAKE)

dbhandler :
	cd dbhandler && $(MAKE)

ruddercommand :
	cd ruddercommand && $(MAKE)

sailcommand :
	cd sailcommand && $(MAKE)

waypointlist :
	cd waypointlist && $(MAKE)

servocontroller :
	cd servocontroller && $(MAKE)

windsensor :
	cd windsensor && $(MAKE)

gps :
	cd gps && $(MAKE)

httpsync :
	cd httpsync && $(MAKE)

$(FILE) : $(SOURCES) $(HEADERS) $(OBJECTS)
	$(CC) $(SOURCES) $(OBJECTS) $(FLAGS) $(LIBS) -o $(FILE)

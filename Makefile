#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailingrobot
#    -------------------------------------------
#
#######################################################

CC = g++ 
FLAGS = -Wall -pedantic -Werror -std=c++14 
LIBS = -lsqlite3 -lgps -lrt -lwiringPi -lcurl

COURSE = coursecalculation/CourseCalculation.o 
DB = dbhandler/DBHandler.o  dbhandler/JSON.o
COMMAND = ruddercommand/RudderCommand.o sailcommand/SailCommand.o 
MAESTRO = servocontroller/MaestroController.o servocontroller/ServoObject.o
CV7 = CV7/CV7.o
GPS = gps/GPSReader.o gps/MockGPSReader.o
HTTP = httpsync/HTTPSync.o

OBJECTS = $(COURSE) $(DB) $(COMMAND) $(MAESTRO) $(CV7) $(GPS) $(HTTP)
SOURCES = SailingRobot.cpp example.cpp
HEADERS = SailingRobot.h
FILE = sr



all : coursecalculation dbhandler ruddercommand sailcommand servocontroller windsensor gps httpsync $(FILE)

clean :
	cd gps && $(MAKE) clean
	rm -f $(FILE)
	
coursecalculation :
	cd coursecalculation && $(MAKE)

dbhandler :
	cd dbhandler && $(MAKE)

ruddercommand :
	cd ruddercommand && $(MAKE)

sailcommand :
	cd sailcommand && $(MAKE)

servocontroller :
	cd servocontroller && $(MAKE)

windsensor :
	cd CV7 && $(MAKE)

gps :
	cd gps && $(MAKE)

httpsync :
	cd httpsync && $(MAKE)

$(FILE) : $(SOURCES) $(HEADERS) $(OBJECTS)
	$(CC) $(SOURCES) $(OBJECTS) $(FLAGS) $(LIBS) -o $(FILE)

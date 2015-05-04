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

COMPASS = Compass/Compass.o Compass/MockCompass.o Compass/Utility.o Compass/HMC6343.o
COURSE = coursecalculation/CourseCalculation.o 
DB = dbhandler/DBHandler.o  dbhandler/JSON.o
COMMAND = ruddercommand/RudderCommand.o sailcommand/SailCommand.o 
MAESTRO = servocontroller/MaestroController.o servocontroller/MockMaestroController.o servocontroller/ServoObject.o servocontroller/MockServoObject.o
CV7 = CV7/Windsensor.o CV7/MockWindsensor.o CV7/UtilityLibrary.o CV7/CV7.o
GPS = gps/GPSReader.o gps/MockGPSReader.o
HTTP = httpsync/HTTPSync.o
XML_LOG = xmlparser/pugi.o xmlparser/XML_log.o


OBJECTS = $(COMPASS) $(COURSE) $(DB) $(COMMAND) $(MAESTRO) $(CV7) $(GPS) $(HTTP) $(XML_LOG)
SOURCES = SailingRobot.cpp main.cpp
HEADERS = SailingRobot.h
FILE = sr
MAKE = make



all : runall $(FILE)

runall : Compass coursecalculation dbhandler ruddercommand sailcommand servocontroller CV7 gps httpsync xmlparser

clean :
	cd Compass && $(MAKE) clean
	cd coursecalculation && $(MAKE) clean
	cd dbhandler && $(MAKE) clean
	cd ruddercommand && $(MAKE) clean
	cd sailcommand && $(MAKE) clean
	cd servocontroller && $(MAKE) clean
	cd CV7 && $(MAKE) clean
	cd gps && $(MAKE) clean
	cd httpsync && $(MAKE) clean
	cd xmlparser && $(MAKE) clean
	rm -f $(FILE)
	
Compass :
	$(MAKE) -C ./Compass
	

	
coursecalculation :
	$(MAKE) -C ./coursecalculation

#Needed for proper subfolder make writing
.PHONY : Compass runall coursecalculation dbhandler ruddercommand sailcommand servocontroller CV7 gps httpsync xmlparser 
	
dbhandler :
	$(MAKE) -C ./dbhandler


ruddercommand :
	$(MAKE) -C ./ruddercommand


sailcommand :
	$(MAKE) -C ./sailcommand


servocontroller :
	$(MAKE) -C ./servocontroller


CV7 :
	$(MAKE) -C ./CV7


gps :
	$(MAKE) -C ./gps


httpsync :
	$(MAKE) -C ./httpsync

xmlparser :
	$(MAKE) -C ./xmlparser

$(FILE) : $(SOURCES) $(HEADERS) $(OBJECTS)
	$(CC) $(SOURCES) $(OBJECTS) $(FLAGS) $(LIBS) -o $(FILE)

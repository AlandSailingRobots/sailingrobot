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
LIBS = -lsqlite3 -lgps -lrt -lwiringPi -lcurl -lpthread -I$(SAILINGROBOTS_HOME)
LIBS_BOOST = -lboost_system -lboost_log -lboost_thread -I$(SAILINGROBOTS_HOME)


COMPASS = Compass/Compass.o Compass/MockCompass.o Compass/HMC6343.o
COURSE = coursecalculation/CourseCalculation.o coursecalculation/CourseMath.o 
DB = dbhandler/DBHandler.o  dbhandler/JSON.o
COMMAND = ruddercommand/RudderCommand.o sailcommand/SailCommand.o 
MAESTRO = servocontroller/MaestroController.o servocontroller/MockMaestroController.o servocontroller/ServoObject.o servocontroller/MockServoObject.o
CV7 = CV7/Windsensor.o CV7/MockWindsensor.o CV7/UtilityLibrary.o CV7/CV7.o
GPS = gps/GPSReader.o gps/MockGPSReader.o
HTTP = httpsync/HTTPSync.o
XML_LOG = xmlparser/pugi.o xmlparser/XML_log.o
XBEE = xBee/xBee.o
THREAD = thread/SystemState.o thread/ExternalCommand.o thread/ThreadRAII.o
WAYPOINTROUTING = waypointrouting/WaypointRouting.o


OBJECTS = $(COMPASS) $(COURSE) $(DB) $(COMMAND) $(MAESTRO) $(CV7) $(GPS) \
		  $(HTTP) $(XML_LOG) $(XBEE) $(THREAD) $(WAYPOINTROUTING) \
		  logger/Logger.o utility/Utility.o
SOURCES = SailingRobot.cpp main.cpp xBeeSync.cpp GPSupdater.cpp WindsensorController.cpp
HEADERS = SailingRobot.h main.h xBeeSync.h GPSupdater.h WindsensorController.h
FILE = sr
MAKE = make

#Needed for proper subfolder make writing
.PHONY : Compass runall coursecalculation dbhandler ruddercommand \
		 sailcommand servocontroller CV7 gps httpsync xmlparser thread \
		 logger utility waypointrouting


all : runall $(FILE)

runall : Compass coursecalculation dbhandler ruddercommand sailcommand \
		 servocontroller CV7 gps httpsync xmlparser thread logger utility \
		 waypointrouting

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
	cd xBee && $(MAKE) clean
	cd thread && $(MAKE) clean
	cd logger && $(MAKE) clean
	cd utility && $(MAKE) clean
	cd waypointrouting && $(MAKE) clean
	rm -f $(FILE)
	
Compass :
	$(MAKE) -C ./Compass
	
coursecalculation :
	$(MAKE) -C ./coursecalculation

	
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

xbee :
	$(MAKE) -C ./xbee

thread :
	$(MAKE) -C ./thread

waypointrouting :
	$(MAKE) -C ./waypointrouting

logger:
	$(MAKE) -C ./logger

utility:
	$(MAKE) -C utility

$(FILE) : $(SOURCES) $(HEADERS) $(OBJECTS)
	$(CC) $(SOURCES) $(OBJECTS) $(FLAGS) $(LIBS) $(LIBS_BOOST) -o $(FILE)
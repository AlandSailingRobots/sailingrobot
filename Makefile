#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailingrobot
#    -------------------------------------------
#
#######################################################


#######################################################
# TOOLCHAINS
#######################################################


# Options include:
#		linux_local = For a local linux machine (MOCK objects used)
#		raspi_cc = For cross compiling to the PI on a linux machine
#		raspi_local = For compiling on the PI itself
export TOOLCHAIN = linux-local
C_TOOLCHAIN = 0


#######################################################
# FILES
#######################################################

# Directories
export BUILD_DIR = build
SRC_DIR = ./
OUTPUT_DIR = ./

TEST_MAKEFILE = ./Makefile.tests

# External Libraries

JSON = 					libs/json

# Sources

XBEE = 					xBee/xBeeSync.cpp xBee/xBee.cpp

BEHAVIOURCLASS = 	behaviourclass/RoutingBehaviour.cpp  behaviourclass/WaypointBehaviour.cpp behaviourclass/LineFollowBehaviour.cpp

ANALOGARDUINO = 		AnalogArduino/AnalogArduino.cpp AnalogArduino/MockAnalogArduino.cpp AnalogArduino/AR_UNO.cpp

COMPASS = 				Compass/Compass.cpp Compass/MockCompass.cpp Compass/HMC6343.cpp

I2CCONTROLLER = 		i2ccontroller/I2CController.cpp

POSITION = 				utility/Position.cpp utility/MockPosition.cpp utility/RealPosition.cpp

COURSE = 				coursecalculation/CourseCalculation.cpp coursecalculation/CourseMath.cpp

DB = 					dbhandler/DBHandler.cpp

COMMAND = 				ruddercommand/RudderCommand.cpp sailcommand/SailCommand.cpp

MAESTRO = 				servocontroller/MaestroController.cpp servocontroller/MockMaestroController.cpp servocontroller/ServoObject.cpp servocontroller/SensorObject.cpp servocontroller/MockServoObject.cpp

CV7 = 					CV7/Windsensor.cpp CV7/MockWindsensor.cpp CV7/UtilityLibrary.cpp CV7/CV7.cpp

GPS = 					gps/GPSReader.cpp gps/MockGPSReader.cpp

HTTP = 					httpsync/HTTPSync.cpp

XML_LOG = 				xmlparser/pugi/pugixml.cpp xmlparser/src/xml_log.cpp

THREAD = 				thread/SystemState.cpp thread/ExternalCommand.cpp thread/ThreadRAII.cpp

WAYPOINTROUTING = 		waypointrouting/WaypointRouting.cpp waypointrouting/Commands.cpp waypointrouting/TackAngle.cpp

WINDVANECONTROLLER = 	windvanecontroller/WindVaneController.cpp

SRC_MAIN = main.cpp

SRC = 	GPSupdater.cpp SailingRobot.cpp WindsensorController.cpp logger/Logger.cpp utility/Utility.cpp utility/Timer.cpp $(XBEE) \
		$(ANALOGARDUINO) $(COMPASS) $(I2CCONTROLLER) $(POSITION) $(COURSE) $(DB) $(COMMAND) $(MAESTRO) $(CV7) $(GPS) $(HTTP) \
		$(XML_LOG) $(THREAD) $(WAYPOINTROUTING) $(WINDVANECONTROLLER) $(BEHAVIOURCLASS)

#SOURCES = $(addprefix src/, $(SRC))

# Includes

export INC = -I./ -I./libs

INC = -I./ -I./libs -I./libs/wiringPi/wiringPi

WIRING_PI_PATH = ./libs/wiringPi/wiringPi/
WIRING_PI_STATIC = ./libs/wiringPi/wiringPi/libwiringPi.so.2.32

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o)) $(BUILD_DIR)/AnalogArduino/libmyWiringI2C.so
OBJECT_MAIN = $(addprefix $(BUILD_DIR)/, $(SRC_MAIN:.cpp=.o))

# Target Output
EXECUTABLE = sr
export OBJECT_FILE = $(BUILD_DIR)/objects.tmp


#######################################################
# TOOLS
#######################################################


export CFLAGS = -Wall -g -o2
export CPPFLAGS = -g -Wall -pedantic -Werror -std=c++11

export LIBS = -lsqlite3 -lgps -lrt -lcurl -lpthread

ifeq ($(TOOLCHAIN),raspi_cc)
C_TOOLCHAIN = 0
CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++
SIZE = arm-linux-gnueabihf-size
else
C_TOOLCHAIN = 1
CC = gcc
CXX = g++
SIZE = size
endif

export CC
export CXX

export MKDIR_P = mkdir -p


#######################################################
# Rules
#######################################################

.PHONY: clean clean_tests

all: $(EXECUTABLE) stats

# Builds the intergration test, requires the whole system to be built before
build_tests: $(OBJECTS) $(EXECUTABLE)
	@echo Building tests...
	@$(MAKE) -f $(TEST_MAKEFILE)

clean_tests:
	@echo Cleaning tests...
	@$(MAKE) -f $(TEST_MAKEFILE) clean

#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)

$(WIRING_PI_STATIC):
	$(MAKE) -C $(WIRING_PI_PATH)

# Link and build
$(EXECUTABLE) : $(BUILD_DIR) $(OBJECTS) $(WIRING_PI_STATIC) $(OBJECT_MAIN)
	rm -f $(OBJECT_FILE)
	@echo Linking object files
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	$(CXX) $(LDFLAGS) -Wl,-rpath,./ @$(OBJECT_FILE) $(WIRING_PI_STATIC) $(OBJECT_MAIN) -o $@ $(LIBS) $(LIBS_BOOST)
	@echo Built using toolchain: $(TOOLCHAIN)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC) -o ./$@ $< -DTOOLCHAIN=$(TOOLCHAIN) $(LIBS) $(LIBS_BOOST)

 # Compile C files into the build folder
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo Compiling C File: $@
	@$(C) -c $(CFLAGS) $(INC) -o $@ $ -DTOOLCHAIN=$(C_TOOLCHAIN)

#SPECIAL COMPILATION FOR mywiringI2C.cpp to be overload when doing simulation AnalogArduino/myWiringI2C.cpp
$(BUILD_DIR)/AnalogArduino/libmyWiringI2C.so: $(SRC_DIR)/AnalogArduino/myWiringI2C.cpp
	@echo Compiling CPP File to Shared library: $(SRC_DIR)/AnalogArduino/myWiringI2C.cpp
	$(CXX) -c -fPIC $(CPPFLAGS) $(INC) $(SRC_DIR)/AnalogArduino/myWiringI2C.cpp -o $(BUILD_DIR)/AnalogArduino/myWiringI2C.o -DTOOLCHAIN=$(TOOLCHAIN) $(LIBS)
	$(CXX) -shared -Wl,-soname,libmyWiringI2C.so -o $(BUILD_DIR)/AnalogArduino/libmyWiringI2C.so $(BUILD_DIR)/AnalogArduino/myWiringI2C.o -ldl
	cp $(BUILD_DIR)/AnalogArduino/libmyWiringI2C.so  $(SRC_DIR)/

#####################################################################
# Tool Rules

stats:$(EXECUTABLE)
	@echo Final executable size:
	$(SIZE) $(EXECUTABLE)

clean: clean_tests
	@echo Removing existing object files and executable
	@rm -f -r $(BUILD_DIR)
	@rm -f $(EXECUTABLE)
	@echo DONE

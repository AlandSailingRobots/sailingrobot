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
TOOLCHAIN = linux_local 
C_TOOLCHAIN = 1


#######################################################
# FILES
#######################################################

# Directories
BUILD_DIR = build
SRC_DIR = ./
OUTPUT_DIR = ./

# Sources

XBEE = 					xBee/xBeeSync.cpp xBee/xBee.cpp

ANALOGARDUINO = 		AnalogArduino/AnalogArduino.cpp AnalogArduino/MockAnalogArduino.cpp AnalogArduino/AR_UNO.cpp AnalogArduino/myWiringI2C.cpp

COMPASS = 				Compass/Compass.cpp Compass/MockCompass.cpp Compass/HMC6343.cpp

I2CCONTROLLER = 		i2ccontroller/I2CController.cpp

POSITION = 				utility/Position.cpp utility/MockPosition.cpp utility/RealPosition.cpp

COURSE = 				coursecalculation/CourseCalculation.cpp coursecalculation/CourseMath.cpp

DB = 					dbhandler/DBHandler.cpp

COMMAND = 				ruddercommand/RudderCommand.cpp sailcommand/SailCommand.cpp

MAESTRO = 				servocontroller/MaestroController.cpp servocontroller/MockMaestroController.cpp servocontroller/ServoObject.cpp servocontroller/MockServoObject.cpp

CV7 = 					CV7/Windsensor.cpp CV7/MockWindsensor.cpp CV7/UtilityLibrary.cpp CV7/CV7.cpp

GPS = 					gps/GPSReader.cpp gps/MockGPSReader.cpp

HTTP = 					httpsync/HTTPSync.cpp

XML_LOG = 				xmlparser/pugi.cpp xmlparser/XML_log.cpp

THREAD = 				thread/SystemState.cpp thread/ExternalCommand.cpp thread/ThreadRAII.cpp

WAYPOINTROUTING = 		waypointrouting/WaypointRouting.cpp waypointrouting/Commands.cpp waypointrouting/TackAngle.cpp

WINDVANECONTROLLER = 	windvanecontroller/WindVaneController.cpp


SRC = GPSupdater.cpp SailingRobot.cpp WindsensorController.cpp logger/Logger.cpp utility/Utility.cpp utility/Timer.cpp $(XBEE) \
		$(ANALOGARDUINO) $(COMPASS) $(I2CCONTROLLER) $(POSITION) $(COURSE) $(DB) $(COMMAND) $(MAESTRO) $(CV7) $(GPS) $(HTTP) \
		$(XML_LOG) $(THREAD) $(WAYPOINTROUTING) $(WINDVANECONTROLLER)

SOURCES = $(addprefix src/, $(SRC))

# Includes

INC = -I./


# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

# Target Output
EXECUTABLE = $(OUTPUT_DIR)sr
OBJECT_FILE = $(BUILD_DIR)/objects.tmp


#######################################################
# TOOLS
#######################################################


CFLAGS = -Wall -g -o2
CPPFLAGS = -g -Wall -pedantic -Werror -std=c++14

LIBS = -lsqlite3 -lgps -lrt -lwiringPi -lcurl -lpthread
LIBS_BOOST = -lboost_system -lboost_log -lboost_thread

ifeq ($(TOOLCHAIN), raspi_cc)
C_TOOLCHAIN = 0
CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++
SIZE = arm-linux-gnueabihf-size
else ifeq ($(TOOLCHAIN), linux_local)
C_TOOLCHAIN = 1
CC = gcc
CXX = g++
SIZE = size
endif

MKDIR_P = mkdir -p


#######################################################
# Rules
#######################################################

.PHONY: clean

all: $(EXECUTABLE) stats

#  Create the directories needed
$(BUILD_DIR):
	$(MKDIR_P) $(BUILD_DIR)

# Link and build
$(EXECUTABLE) : $(BUILD_DIR) $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo Linking object files
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -o $@
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


#####################################################################
# Tool Rules

stats:$(EXECUTABLE)
	@echo Final executable size:
	@$(SIZE) $(EXECUTABLE)

clean:
	rm -f -r $(BUILD_DIR)
	rm -f $(EXECUTABLE)
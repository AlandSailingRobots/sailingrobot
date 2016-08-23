#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailingrobot
#    -------------------------------------------
#
#######################################################


#######################################################
# Extra Build Options
#######################################################


# Enables OpenCV color detection if it is set
USE_OPENCV = 0

# Allows the building of preset lists of nodes
# DEFAULT: THis is the default janet set of nodes
# SIM: THis is the default janet but with the sensor nodes being replaced with the simulator node
# WRSC: This is the set of nodes used for WRSC2016
# XBEE_REMOTE: The xbee remote tool
TARGET = DEFAULT


#######################################################
# TOOLCHAINS
#######################################################


# Options include:
#		linux_local = For a local linux machine
#		raspi_cc = For cross compiling to the PI on a linux machine
#		raspi_local = For compiling on the PI itself
# 		win = for windows
export TOOLCHAIN = linux-local
C_TOOLCHAIN = 0
USE_SIM = 0

#######################################################
# Directories
#######################################################

export BUILD_DIR 		= build
SRC_DIR 				= ./
OUTPUT_DIR 				= ./

#######################################################
# Build targets
#######################################################

UNIT_TEST 				= ./unit-tests.run
HARDWARE_TEST 			= ./hardware-tests.run
EXECUTABLE 				= sr

#######################################################
# FILES TO BUILD
#######################################################

# List of Sources

SRC_MAIN 				= main.cpp

SRC_MESSAGES			= Messages/MessageSerialiser.cpp Messages/MessageDeserialiser.cpp

SRC_CORE				= MessageBus/MessageBus.cpp Nodes/ActiveNode.cpp $(SRC_MESSAGES) utility/CourseCalculation.cpp utility/CourseMath.cpp dbhandler/DBHandler.cpp dbhandler/DBLogger.cpp utility/Utility.cpp utility/Timer.cpp

SRC_CORE_SAILING		= waypointrouting/RudderCommand.cpp waypointrouting/SailCommand.cpp waypointrouting/WaypointRouting.cpp waypointrouting/Commands.cpp waypointrouting/TackAngle.cpp

SRC_CORE_NODES			= Nodes/MessageLoggerNode.cpp Nodes/WaypointMgrNode.cpp Nodes/VesselStateNode.cpp Nodes/RoutingNode.cpp Nodes/LineFollowNode.cpp 

SRC_COMMON				= utility/SysClock.cpp SystemServices/Logger.cpp

SRC_SENSOR_NODES		= Nodes/CV7Node.cpp Nodes/HMC6343Node.cpp Nodes/GPSDNode.cpp Nodes/ArduinoNode.cpp

SRC_ACTUATOR_NODE		= SystemServices/MaestroController.cpp Nodes/ActuatorNode.cpp

SRC_NETWORK_XBEE		= Network/DataLink.cpp Network/XbeePacketNetwork.cpp

SRC_NETWORK_XBEE_LINUX 	= $(SRC_NETWORK_XBEE) Network/LinuxSerialDataLink.cpp Nodes/XbeeSyncNode.cpp

SRC_NETWORK_HTTP_SYNC	= Nodes/HTTPSyncNode.cpp

SRC_I2CCONTROLLER 		= i2ccontroller/I2CController.cpp

SRC_LIDAR_LITE			= Nodes/lidarLite/lidarLite.cpp Nodes/lidarLite/lidarLiteNode.cpp
SRC_OPENCV_CV 			= Nodes/obstacledetection/colorDetectionNode.cpp Nodes/obstacledetection/colorDetectionUtility.cpp 

SRC_SIMULATOR			= Nodes/SimulationNode.cpp

# Default Janet build
ifeq($(TARGET),DEFAULT)

SRC 					= $(SRC_CORE) $(SRC_CORE_SAILING) $(SRC_CORE_NODES) $(SRC_COMMON) $(SRC_SENSOR_NODES) $(SRC_ACTUATOR_NODE) $(SRC_NETWORK_XBEE_LINUX) $(SRC_NETWORK_HTTP_SYNC) $(SRC_I2CCONTROLLER) $(SRC_MAIN)
endif

# SIM build
ifeq($(TARGET),SIM)

SRC 					= $(SRC_CORE) $(SRC_CORE_SAILING) $(SRC_CORE_NODES) $(SRC_COMMON) $(SRC_SIMULATOR) $(SRC_NETWORK_XBEE_LINUX) $(SRC_NETWORK_HTTP_SYNC) $(SRC_MAIN)

endif

# WRSC2016 build
ifeq($(TARGET),WRSC)

SRC 					= $(SRC_CORE) $(SRC_CORE_SAILING) $(SRC_CORE_NODES) $(SRC_COMMON) $(SRC_WRSC_NODES) $(SRC_ACTUATOR_NODE) $(SRC_MAIN) $(SRC_NETWORK_WIFI_UDP)
# $(SRC_OPENCV_CV) Get working properly

endif

# Xbee Remote tool
ifeq( $(TARGET), XBEE_REMOTE)
ifeq ($(TOOLCHAIN),win)

SRC						= $(SRC_COMMON) $(SRC_NETWORK_XBEE) $(SRC_MESSAGES)

else

SRC						= $(SRC_NETWORK_XBEE_LINUX) $(SRC_COMMON) $(SRC_MESSAGES)

endif
endif

WIRING_PI = libwiringPi.so
WIRING_PI_PATH = ./libs/wiringPi/wiringPi/
WIRING_PI_STATIC = ./libs/wiringPi/wiringPi/libwiringPi.so.2.32

# Includes


export INC = -I./ -I./libs -I./libs/wiringPi/wiringPi -I.\


# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))
OBJECT_MAIN = $(addprefix $(BUILD_DIR)/, $(SRC_MAIN:.cpp=.o))

# Target Output
export OBJECT_FILE = $(BUILD_DIR)/objects.tmp


#######################################################
# TOOLS
#######################################################


ifeq ($(TOOLCHAIN),win)

export CFLAGS = -Wall -g -o2
export CPPFLAGS = -g -Wall -pedantic -Werror -std=gnu++14

export LIBS = 

else

export CFLAGS = -Wall -g -o2
export CPPFLAGS = -g -Wall -pedantic -Werror -std=gnu++14


export LIBS = -lsqlite3 -lgps -lrt -lcurl -lpthread
endif

ifeq ($(USE_OPENCV), 1)
LIBS += `pkg-config --libs opencv`
endif

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

DEFINES = -DTOOLCHAIN=$(TOOLCHAIN) -DSIMULATION=$(USE_SIM) -DSE_OPENCV_COLOR_DETECTION=$(USE_OPENCV)


#######################################################
# Rules
#######################################################

.PHONY: clean

all: $(EXECUTABLE) stats


simulation:
	make USE_SIM=1 -j4

# Builds the intergration test, requires the whole system to be built before
build_tests: $(OBJECTS) $(EXECUTABLE)
	@echo Building tests...
	$(MAKE) -C tests
	$(CXX) $(CPPFLAGS) tests/runner.o @$(OBJECT_FILE) -Wl,-rpath=./ ./libwiringPi.so -o $(UNIT_TEST) $(LIBS)
	$(CXX) $(CPPFLAGS) tests/runnerHardware.o @$(OBJECT_FILE) -Wl,-rpath=./ ./libwiringPi.so -o $(HARDWARE_TEST) $(LIBS)

xbee_remote: $(OBJECTS) $(WIRING_PI)
	"$(MAKE)" -C XbeeRemote

#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)

$(WIRING_PI):
	$(MAKE) -C $(WIRING_PI_PATH)
	@mv $(WIRING_PI_STATIC) ./libwiringPi.so

# Link and build
$(EXECUTABLE) : $(BUILD_DIR) $(OBJECTS) $(WIRING_PI) $(OBJECT_MAIN)
	rm -f $(OBJECT_FILE)
	@echo Linking object files
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) ./libwiringPi.so $(OBJECT_MAIN) -Wl,-rpath=./ -o $@ $(LIBS) $(LIBS_BOOST)
	@echo Built using toolchain: $(TOOLCHAIN)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@

	@$(CXX) -c $(CPPFLAGS) $(INC) -o ./$@ $< $(DEFINES) $(LIBS) $(LIBS_BOOST)

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

clean:
	@echo Removing existing object files and executable
	@rm -f -r $(BUILD_DIR)
	@rm -f $(EXECUTABLE)
	"$(MAKE)" -C XbeeRemote clean
	
	@echo DONE

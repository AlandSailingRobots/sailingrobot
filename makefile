###############################################################################
#
# Control System Makefile
#
# Targets:
#   * line-follow: Builds the line following code
#   * dev-lnm: Builds the local navigation module
#
# External Variables
#   * USE_SIM: Indicates if the simulator is to be used, 0 for off, 1 for on.
#
# Example
#   For the local navigation module with the simulator
#   make dev-lnm USE_SIM=1
#
# For the default code
#   make
#
###############################################################################


export USE_SIM = 0
TOOLCHAIN = 0


###############################################################################
# Tools
###############################################################################


export CPPFLAGS                = -g -Wall -pedantic -Werror -std=gnu++14
export LIBS                    = -lsqlite3 -lgps -lrt -lcurl -lpthread

ifeq ($(TOOLCHAIN),1)
export CC                      = arm-linux-gnueabihf-gcc
export CXX                     = arm-linux-gnueabihf-g++
export SIZE                    = arm-linux-gnueabihf-size
else
export CC                      = gcc
export CXX                     = g++
export SIZE                    = size
endif

export MKDIR_P          = mkdir -p

export DEFINES          = -DTOOLCHAIN=$(TOOLCHAIN) -DSIMULATION=$(USE_SIM)


###############################################################################
# Folder Paths
###############################################################################

export SRC_DIR			= ./
export BUILD_DIR        = build
export EXEC_DIR         = ./
export INC_DIR          = -I./ -I./libs -I./libs/wiringPi/wiringPi

LNM_DIR                 = Nodes/LocalNavigationModule


###############################################################################
# Files
###############################################################################

# Target Output
export EXECUTABLE           = sr
export OBJECT_FILE          = $(BUILD_DIR)/objects.tmp


export MAIN_SRC             = main.cpp
export MAIN_LNM_SRC         = main_lnm.cpp

export MESSAGE_BUS_SRC      = MessageBus/MessageBus.cpp Nodes/ActiveNode.cpp Messages/MessageSerialiser.cpp \
                            Messages/MessageDeserialiser.cpp

export LNM_SRC              = $(LNM_DIR)/ASRCourseBallot.cpp $(LNM_DIR)/ASRArbiter.cpp \
                            $(LNM_DIR)/LocalNavigationModule.cpp Nodes/LowLevelController.cpp \
                            $(LNM_DIR)/Voters/WaypointVoter.cpp $(LNM_DIR)/Voters/WindVoter.cpp  \
                            $(LNM_DIR)/Voters/ChannelVoter.cpp

export LINE_FOLLOW_SRC      = Nodes/LineFollowNode.cpp waypointrouting/RudderCommand.cpp \
                            waypointrouting/SailCommand.cpp Nodes/MessageLoggerNode.cpp

export NETWORK_SRC          = Network/TCPServer.cpp Nodes/VesselStateNode.cpp

export HARDWARE_NODES_SRC   = Nodes/CV7Node.cpp Nodes/HMC6343Node.cpp Nodes/GPSDNode.cpp \
                            Nodes/ActuatorNode.cpp Nodes/ArduinoNode.cpp

export SYSTEM_SERVICES_SRC  = SystemServices/Logger.cpp SystemServices/SysClock.cpp SystemServices/Timer.cpp \
                            dbhandler/DBHandler.cpp dbhandler/DBLogger.cpp SystemServices/CANService.cpp \
														SystemServices/CANPGNReceiver.cpp SystemServices/CANWindsensorNode.cpp

export HARDWARE_SERVICES_SRC = SystemServices/MaestroController.cpp i2ccontroller/I2CController.cpp

export MATH_SRC             = Math/CourseCalculation.cpp Math/CourseMath.cpp Math/Utility.cpp

export SIMULATOR_SRC        = Nodes/SimulationNode.cpp

export CORE_SRC             = Nodes/WaypointMgrNode.cpp $(MESSAGE_BUS_SRC) $(NETWORK_SRC) \
                            $(SYSTEM_SERVICES_SRC) $(MATH_SRC)

export HTTP_SYNC_SRC        = Nodes/HTTPSyncNode.cpp

# TODO: Break down for Xbee Remote
export XBEE_NETWORK_SRC     = Network/DataLink.cpp Network/LinuxSerialDataLink.cpp Network/XbeePacketNetwork.cpp \
                            xBee/Xbee.cpp Nodes/XbeeSyncNode.cpp


###############################################################################
# Rules
###############################################################################

.PHONY: clean

all: $(EXECUTABLE) stats

dev-lnm: $(BUILD_DIR) $(WIRING_PI)
	$(MAKE) -f dev-lnm.mk -j

line-follow: $(BUILD_DIR) $(WIRING_PI)
	$(MAKE) -f line-follow.mk -j4

#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)

$(WIRING_PI):
	$(MAKE) -C $(WIRING_PI_PATH)
	@mv $(WIRING_PI_STATIC) ./libwiringPi.so

clean:
	@echo Removing existing object files and executable
	-@rm -rd $(BUILD_DIR)
	-@rm $(EXECUTABLE)
	$(MAKE) -C tests clean

	@echo DONE

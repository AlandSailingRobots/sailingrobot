###############################################################################
#
# Master makefile of the Navigation System
#
#   Type 'make help' to display help text, with targets and external variables
#
# Usage: make [target] [ext variable 1] [ext variable 2]
#
#   Targets:
#		* ASPire: Build for ASPire
#		* Janet: Build for Janet
#
#   External Variables
#   	* USE_SIM: Indicates if the simulator is to be used, 0 for off, 1 for on.
#		* USE_LNM: 1: Local Navigation Module (voter system), 0: Line-follow (default)
#
#   Example
#   	Build the Janet navigation system with simulator interface and local navigation module
#   	'make Janet USE_SIM=1 USE_LNM=1'
#
# For the default code:   'make'
#
###############################################################################

TOOLCHAIN = 0
export USE_SIM = 0
export USE_LNM = 0

## OPENCV Libs for camera processing (depends if we compile for RaspberryPi or not)
# To detect if we are not compiling for the RaspberryPi
uname_m := $(shell uname -m)
ifneq ($(uname_m),armv7l)
OPENCVCFLAGS = `pkg-config opencv --cflags`
OPENCVLIBS = `pkg-config opencv --libs`
else #Compiling on RaspberryPi
OPENCVCFLAGS =
OPENCVLIBS = -lopencv_core -lopencv_videoio -lopencv_imgproc
endif

###############################################################################
# Tools
###############################################################################


export CPPFLAGS             = -g -Wall -pedantic -Werror -std=gnu++14 -DBOOST_LOG_DYN_LINK -Wno-psabi $(OPENCVCFLAGS)

export LIBS                 = -lsqlite3 -lgps -lrt -lcurl -lwiringPi -lncurses $(OPENCVLIBS) \
                              -lboost_log -lboost_thread -lboost_system -lboost_filesystem -lpthread 
                              # keep -lpthread after the boost libs, it can fail otherwise 


ifeq ($(TOOLCHAIN),1)
    export CC               = arm-linux-gnueabihf-gcc
    export CXX              = arm-linux-gnueabihf-g++
    export SIZE             = arm-linux-gnueabihf-size
else
    export CC               = gcc
    export CXX				= g++
    export SIZE             = size
endif

export MKDIR_P				= mkdir -p

export DEFINES          	= -DTOOLCHAIN=$(TOOLCHAIN) -DSIMULATION=$(USE_SIM) \
								-DLOCAL_NAVIGATION_MODULE=$(USE_LNM)


###############################################################################
# Folder Paths
###############################################################################

export SRC_DIR				= ./
export BUILD_DIR        	= build
export EXEC_DIR         	= ./

export INC_DIR         	 	= -I./ -I./Libs -I./Libs/json/include   

LNM_DIR                 	= Navigation/LocalNavigationModule


###############################################################################
# Target Output
###############################################################################

export EXECUTABLE           = sr
export UNIT_TEST_EXEC 		= unit-tests.run
export UNIT_TEST_HW_EXEC 	= unit-tests-hardware.run
export INTEGRATION_TEST_EXEC_ASPIRE = integration-tests-ASPire.run
export MARINE_SENSOR_INTEGRATION_TEST_EXEC = marine_sensor-integration-tests.run
export HTTP_SYNC_TEST_EXEC	= HTTPSync-test.run
export AIS_TEST_EXEC		= ais-integration-tests.run
export CURRENT_SENSOR_INTEGRATION_TEST_EXEC = current_sensor-integration-tests.run

export OBJECT_FILE          = $(BUILD_DIR)/objects.tmp


###############################################################################
# Source Files
###############################################################################

# Core
DATABASE_SRC				= Database/DBHandler.cpp Database/DBLogger.cpp Database/DBLoggerNode.cpp

HTTP_SYNC_SRC        		= HTTPSync/HTTPSyncNode.cpp

LOW_LEVEL_CONTROLLERS 		= LowLevelControllers/CourseRegulatorNode.cpp LowLevelControllers/SailControlNode.cpp \
								LowLevelControllers/WingsailControlNode.cpp

MATH_SRC             		= Math/CourseCalculation.cpp Math/CourseMath.cpp Math/Utility.cpp

MESSAGE_BUS_SRC      		= MessageBus/MessageBus.cpp MessageBus/ActiveNode.cpp \
                            	MessageBus/MessageSerialiser.cpp MessageBus/MessageDeserialiser.cpp

NETWORK_SRC          		= Network/TCPServer.cpp

NAVIGATION_SRC				= Navigation/WaypointMgrNode.cpp

SYSTEM_SERVICES_SRC  		= SystemServices/Logger.cpp SystemServices/SysClock.cpp SystemServices/Timer.cpp

WORLD_STATE_SRC				= WorldState/VesselStateNode.cpp WorldState/StateEstimationNode.cpp \
								WorldState/WindStateNode.cpp WorldState/CameraProcessingUtility.cpp

DATA_COLLECTION_SRC			= DataCollection/DataCollectionMgrNode.cpp

export CORE_SRC 			= $(DATA_COLLECTION_SRC) $(DATABASE_SRC) $(HTTP_SYNC_SRC) $(LOW_LEVEL_CONTROLLERS) $(MATH_SRC) \
								$(MESSAGE_BUS_SRC) $(NETWORK_SRC) $(NAVIGATION_SRC) $(SYSTEM_SERVICES_SRC) \
								$(WORLD_STATE_SRC) $(CANBUS_COMMON_SRC)

# Local navigation
export LINE_FOLLOW_SRC      = Navigation/LineFollowNode.cpp

export LNM_SRC              = $(LNM_DIR)/ASRCourseBallot.cpp $(LNM_DIR)/ASRArbiter.cpp \
                            	$(LNM_DIR)/LocalNavigationModule.cpp \
                            	$(LNM_DIR)/Voters/WaypointVoter.cpp $(LNM_DIR)/Voters/WindVoter.cpp  \
                            	$(LNM_DIR)/Voters/ChannelVoter.cpp $(LNM_DIR)/Voters/MidRangeVoter.cpp \
								$(LNM_DIR)/Voters/ProximityVoter.cpp $(LNM_DIR)/Voters/CourseVoter.cpp \
								$(LNM_DIR)/VoterTCPDebugger.cpp

# Obstacles detection
export COLLIDABLE_MGR_SRC	= WorldState/CollidableMgr/CollidableMgr.cpp WorldState/AISProcessing.cpp \
							

# Simulator
export SIMULATOR_SRC        = Simulation/SimulationNode.cpp


# Hardware services
export HW_SERVICES_ALL_SRC	= Hardwares/i2ccontroller/I2CController.cpp Hardwares/AtlasScientificController/AtlasScientific.cpp

export CAN_SERVICES_SRC 	= Hardwares/CAN_Services/CANPGNReceiver.cpp Hardwares/CAN_Services/CANService.cpp \
							   	Hardwares/CAN_Services/mcp2515.cpp Hardwares/CAN_Services/MsgFunctions.cpp \
							   	Hardwares/CAN_Services/CANFrameReceiver.cpp

export HW_SERVICES_JANET_SRC = Hardwares/MaestroController/MaestroController.cpp

# Hardware nodes
export HW_NODES_ALL_SRC		= Hardwares/HMC6343Node.cpp Hardwares/GPSDNode.cpp

export HW_NODES_ASPIRE_SRC 	= Hardwares/ActuatorNodeASPire.cpp Hardwares/CANAISNode.cpp Hardwares/CANSolarTrackerNode.cpp\
                            	Hardwares/CANWindsensorNode.cpp Hardwares/CANArduinoNode.cpp \
                            	Hardwares/CANMarineSensorTransmissionNode.cpp Hardwares/CANMarineSensorReceiver.cpp \
                            	Hardwares/CANCurrentSensorNode.cpp 


# ActuatorNode for Janet not compiling, uses removed ActuatorPositionMsg
export HW_NODES_JANET_SRC 	= Hardwares/CV7Node.cpp Hardwares/ArduinoNode.cpp

# XBee network, XbeeSyncNode not compiling, uses removed ActuatorPositionMsg
export XBEE_NETWORK_SRC     = Network/DataLink.cpp Network/LinuxSerialDataLink.cpp Network/XbeePacketNetwork.cpp \
                            	Xbee/Xbee.cpp

# CanbusCommon files
export CANBUS_COMMON_SRC     = Hardwares/CAN_Services/CanBusCommon/CanUtility.cpp Hardwares/CAN_Services/CanBusCommon/CanMessageHandler.cpp


###############################################################################
# Rules
###############################################################################

.PHONY: clean help

## Default, same as make ASPire
all:
	$(MAKE) ASPire

## Build the control system for ASPire
ASPire: $(BUILD_DIR)
	$(eval override CPPFLAGS += -Don_ASPire) # add a preprocessor flag to use in the wind voter for taking into account the tack downwind
	$(MAKE) -f ASPire.mk

## Build the control system for Janet
Janet: $(BUILD_DIR)
	$(MAKE) -f Janet.mk

## Build the unit tests
unit_tests: $(BUILD_DIR)
	$(MAKE) -C Tests
	$(MAKE) -f unit_tests.mk

## Build the integration tests for ASPire
integration_tests_ASPire: $(BUILD_DIR)
	$(MAKE) -f integration_tests_ASPire.mk
	$(MAKE) -f ais_integration_test.mk
	$(MAKE) -f marine_sensor_integration_test.mk
	$(MAKE) -f current_sensor_integration_test.mk


## Build HTTPSync Test
HTTPSync_test: $(BUILD_DIR)
	$(MAKE) -f HTTP_sync_test.mk

#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)


## Remove object files and executables
clean:
	@echo Removing existing object files and executable
	-@rm -rd $(BUILD_DIR)
	-@rm $(EXECUTABLE)
	-@rm $(UNIT_TEST_EXEC)
	-@rm $(UNIT_TEST_HW_EXEC)
	-@rm $(INTEGRATION_TEST_EXEC_ASPIRE)
	-@rm $(INTEGRATION_TEST_EXEC_ASPIRE)
	-@rm $(AIS_TEST_EXEC)
	-@$(MAKE) -C Tests clean
	@echo DONE


## Displays this help text
help:
	@echo -e '\nUsage: make [target] [ext variable 1] [ext variable 2]'
	@echo -e '\nAvailable targets:'

#	$(info Available targets: )
	@awk '/^[a-zA-Z\-\_0-9]+:/ {                    	\
		nb = sub( /^## /, "", helpMsg );              	\
		if(nb == 0) {                                 	\
	  		helpMsg = $$0;                              \
	  		nb = sub( /^[^:]*:.* ## /, "", helpMsg );   \
		}                                             	\
		if (nb)                                       	\
			print "\t" $$1 helpMsg;                     \
	}                                               	\
	{ helpMsg = $$0 }'                              	\
	$(MAKEFILE_LIST) | column -ts:

	@echo -e '\nExternal Variables:'
	@echo -e '\tUSE_SIM = 1:Use with simulator	0: Without (default)'
	@echo -e '\tUSE_LNM = 1:Voter System	0: Line-follow (default)'

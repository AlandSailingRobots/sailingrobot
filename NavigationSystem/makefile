###############################################################################
#
# Navigation System Makefile
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

export USE_SIM = 0
export USE_LNM = 0

TOOLCHAIN = 0


###############################################################################
# Tools
###############################################################################


export CPPFLAGS             = -g -Wall -pedantic -Werror -std=gnu++14
export LIBS                 = -lsqlite3 -lgps -lrt -lcurl -lpthread -lwiringPi -lncurses

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

export INC_DIR         	 	= -I./ -I./Libs

LNM_DIR                 	= Navigation/LocalNavigationModule


###############################################################################
# Files
###############################################################################

# Target Output
export EXECUTABLE           = sr
export UNIT_TEST_EXEC 		= unit-tests.run
export HARDWARE_TEST_EXEC 	= hardware-tests.run
export INTEGRATION_TEST_EXEC_ASPIRE = integration-tests_ASPire.run

export OBJECT_FILE          = $(BUILD_DIR)/objects.tmp

COLLIDABLE_MGR_SRC			= WorldState/CollidableMgr/CollidableMgr.cpp

export LNM_SRC              = $(LNM_DIR)/ASRCourseBallot.cpp $(LNM_DIR)/ASRArbiter.cpp \
                            	$(LNM_DIR)/LocalNavigationModule.cpp LowLevelControllers/LowLevelController.cpp \
                            	$(LNM_DIR)/Voters/WaypointVoter.cpp $(LNM_DIR)/Voters/WindVoter.cpp  \
                            	$(LNM_DIR)/Voters/ChannelVoter.cpp $(LNM_DIR)/Voters/MidRangeVoter.cpp \
								$(LNM_DIR)/Voters/ProximityVoter.cpp $(COLLIDABLE_MGR_SRC)

export LINE_FOLLOW_SRC      = Navigation/LineFollowNode.cpp waypointrouting/RudderCommand.cpp \
                            	waypointrouting/Commands.cpp waypointrouting/SailCommand.cpp \
                            	$(COLLIDABLE_MGR_SRC)

export SIMULATOR_SRC        = Simulation/SimulationNode.cpp

HTTP_SYNC_SRC        		= HTTPSync/HTTPSyncNode.cpp

MESSAGE_BUS_SRC      		= MessageBus/MessageBus.cpp MessageBus/ActiveNode.cpp \
                            	MessageBus/MessageSerialiser.cpp MessageBus/MessageDeserialiser.cpp

MATH_SRC             		= Math/CourseCalculation.cpp Math/CourseMath.cpp Math/Utility.cpp

SYSTEM_SERVICES_SRC  		= SystemServices/Logger.cpp SystemServices/SysClock.cpp SystemServices/Timer.cpp \
                            	DataBase/DBHandler.cpp DataBase/DBLogger.cpp SystemServices/WingsailControl.cpp \
								SystemServices/CourseRegulator.cpp SystemServices/SoftsailControl.cpp \
								DataBase/DBLoggerNode.cpp

NETWORK_SRC          		= Network/TCPServer.cpp

### Base CORE, HARDWARE_NODES, HARDWARE_SERVICES

export CORE_SRC 			= Navigation/WaypointMgrNode.cpp WorldState/VesselStateNode.cpp \
								WorldState/StateEstimationNode.cpp WorldState/WindStateNode.cpp \
								$(MESSAGE_BUS_SRC) $(NETWORK_SRC) $(SYSTEM_SERVICES_SRC) $(MATH_SRC) \
								$(HTTP_SYNC_SRC)

export HARDWARE_SERVICES_SRC = Hardwares/i2ccontroller/I2CController.cpp

export HARDWARE_NODES_SRC	= Hardwares/HMC6343Node.cpp Hardwares/GPSDNode.cpp

### Boat specific variables
### ASPire

export MAIN_ASPIRE			= main_ASPire.cpp

export CAN_HARDWARE_SRC 	= Hardwares/CAN_Services/CANPGNReceiver.cpp Hardwares/CAN_Services/CANService.cpp \
							   	Hardwares/CAN_Services/mcp2515.cpp Hardwares/CAN_Services/MsgFunctions.cpp \
							   	Hardwares/CAN_Services/CANFrameReceiver.cpp

export HARDWARE_NODES_ASPIRE = Hardwares/ActuatorNodeASPire.cpp \
                            	Hardwares/CANFeedbackReceiver.cpp Hardwares/CANWindsensorNode.cpp

export CORE_ASPIRE  	  	= LowLevelControllers/LowLevelControllerNodeASPire.cpp

### Janet

export MAIN_JANET 			= main_janet.cpp

export HARDWARE_SERVICES_JANET = Hardwares/MaestroController/MaestroController.cpp

export HARDWARE_NODES_JANET = Hardwares/CV7Node.cpp \
								Hardwares/ActuatorNode.cpp Hardwares/ArduinoNode.cpp

export CORE_JANET           = LowLevelControllers/LowLevelControllerNodeJanet.cpp

###

# TODO: Break down for Xbee Remote0
export XBEE_NETWORK_SRC     = Network/DataLink.cpp Network/LinuxSerialDataLink.cpp Network/XbeePacketNetwork.cpp \
                            	Xbee/Xbee.cpp Xbee/XbeeSyncNode.cpp


export INTEGRATION_TEST		= Tests/IntegrationTests/ArduinoIntegrationTest.cpp

###############################################################################
# Rules
###############################################################################

.PHONY: clean help

## Default, same as make ASPire
all:
	$(MAKE) ASPire

## Builds the configuration for ASPire
ASPire: $(BUILD_DIR)
	$(MAKE) -f ASPire.mk -j

## Builds the configuration for Janet
Janet: $(BUILD_DIR)
	$(MAKE) -f Janet.mk -j

## Builds the unit_test
unit_tests: $(BUILD_DIR)
	$(MAKE) -C Tests
	$(MAKE) -f tests.mk

## Integration test for ASPire
integration_tests_ASPire: $(BUILD_DIR)
	$(MAKE) -f integration_tests_ASPire.mk


#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)


## Removes object files and executables
clean:
	@echo Removing existing object files and executable
	-@rm -rd $(BUILD_DIR)
	-@rm $(EXECUTABLE)
	$(MAKE) -C Tests clean
	-@rm $(UNIT_TEST_EXEC)
	-@rm $(HARDWARE_TEST_EXEC)
	-@rm $(INTEGRATION_TEST_EXEC_ASPIRE)
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

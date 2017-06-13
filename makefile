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

export LIBS                    = -lsqlite3 -lgps -lrt -lcurl -lpthread -lwiringPi -lncurses

ifeq ($(TOOLCHAIN),1)
export CC                      = arm-linux-gnueabihf-gcc
export CXX                     = arm-linux-gnueabihf-g++
export SIZE                    = arm-linux-gnueabihf-size
else
export CC                      = gcc
export CXX                     = g++
export SIZE                    = size
endif

export MKDIR_P          	= mkdir -p

export DEFINES          	= -DTOOLCHAIN=$(TOOLCHAIN) -DSIMULATION=$(USE_SIM)


###############################################################################
# Folder Paths
###############################################################################

export SRC_DIR				= ./
export BUILD_DIR        	= build
export EXEC_DIR         	= ./
export INC_DIR         	 	= -I./ -I./libs

LNM_DIR                 	= LocalNavigationModule


###############################################################################
# Files
###############################################################################

# Target Output
export EXECUTABLE           = sr
export UNIT_TEST_EXEC 		= unit-tests.run
export HARDWARE_TEST_EXEC 	= hardware-tests.run

export OBJECT_FILE          = $(BUILD_DIR)/objects.tmp


export MAIN_SRC             = main.cpp
export MAIN_LNM_SRC         = main_lnm.cpp

export MESSAGE_BUS_SRC      = MessageBus/MessageBus.cpp Nodes/ActiveNode.cpp Messages/MessageSerialiser.cpp \
                            Messages/MessageDeserialiser.cpp

export COLLIDABLE_MGR_SRC	= CollidableMgr/CollidableMgr.cpp

export LNM_SRC              = $(LNM_DIR)/ASRCourseBallot.cpp $(LNM_DIR)/ASRArbiter.cpp \
                            $(LNM_DIR)/LocalNavigationModule.cpp Nodes/LowLevelController.cpp \
                            $(LNM_DIR)/Voters/WaypointVoter.cpp $(LNM_DIR)/Voters/WindVoter.cpp  \
                            $(LNM_DIR)/Voters/ChannelVoter.cpp $(LNM_DIR)/Voters/ProximityVoter.cpp \
							$(LNM_DIR)/Voters/MidRangeVoter.cpp $(COLLIDABLE_MGR_SRC)

export LINE_FOLLOW_SRC      = Nodes/LineFollowNode.cpp waypointrouting/RudderCommand.cpp \
                            Nodes/MessageLoggerNode.cpp waypointrouting/Commands.cpp waypointrouting/SailCommand.cpp \
                            $(COLLIDABLE_MGR_SRC)

export NETWORK_SRC          = Network/TCPServer.cpp Nodes/VesselStateNode.cpp

export HARDWARE_NODES_SRC   = Nodes/CV7Node.cpp Nodes/HMC6343Node.cpp Nodes/GPSDNode.cpp Nodes/ActuatorNodeASPire.cpp \
                            Nodes/ActuatorNode.cpp Nodes/ArduinoNode.cpp Nodes/CANFeedbackReceiver.cpp Nodes/CANWindsensorNode.cpp

export SYSTEM_SERVICES_SRC  = SystemServices/Logger.cpp SystemServices/SysClock.cpp SystemServices/Timer.cpp \
                            dbhandler/DBHandler.cpp dbhandler/DBLogger.cpp SystemServices/WingsailControl.cpp \
														SystemServices/CourseRegulator.cpp SystemServices/SoftsailControl.cpp \
														Nodes/DBLoggerNode.cpp

export HARDWARE_SERVICES_SRC = HardwareServices/MaestroController/MaestroController.cpp HardwareServices/i2ccontroller/I2CController.cpp \
							   HardwareServices/CAN_Services/CANPGNReceiver.cpp HardwareServices/CAN_Services/CANService.cpp \
							   HardwareServices/CAN_Services/mcp2515.cpp HardwareServices/CAN_Services/MsgFunctions.cpp \
							   HardwareServices/CAN_Services/CANFrameReceiver.cpp

export MATH_SRC             = Math/CourseCalculation.cpp Math/CourseMath.cpp Math/Utility.cpp

export SIMULATOR_SRC        = Nodes/SimulationNode.cpp

export CORE_SRC             = 	Nodes/WaypointMgrNode.cpp Nodes/LowLevelControllerNodeASPire.cpp \
																Nodes/StateEstimationNode.cpp \
							   								Nodes/LowLevelControllerNodeJanet.cpp Nodes/WindStateNode.cpp \
								$(MESSAGE_BUS_SRC) $(NETWORK_SRC) $(SYSTEM_SERVICES_SRC) $(MATH_SRC)

export HTTP_SYNC_SRC        = Nodes/HTTPSyncNode.cpp

# TODO: Break down for Xbee Remote0
export XBEE_NETWORK_SRC     = Network/DataLink.cpp Network/LinuxSerialDataLink.cpp Network/XbeePacketNetwork.cpp \
                            xBee/Xbee.cpp Nodes/XbeeSyncNode.cpp

export INTEGRATION_TEST		= Tests/integration/IntegrationTests/ArduinoIntegrationTest.cpp


###############################################################################
# Rules
###############################################################################

.PHONY: clean $(WIRING_PI)

all: $(EXECUTABLE) stats

dev-lnm: $(BUILD_DIR)
	$(MAKE) -f dev-lnm.mk -j

line-follow: $(BUILD_DIR)
	$(MAKE) -f line-follow.mk -j4

# Builds the intergration test, requires the whole system to be built before
tests: $(BUILD_DIR)
	$(MAKE) -C Tests
	$(MAKE) -f tests.mk

integration_tests: $(BUILD_DIR)
	$(MAKE) -f integration_tests.mk

#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)

clean:
	@echo Removing existing object files and executable
	-@rm -rd $(BUILD_DIR)
	-@rm $(EXECUTABLE)
	-$(MAKE) -C tests clean
	-@rm $(UNIT_TEST_EXEC)
	-@rm $(HARDWARE_TEST_EXEC)
	@echo DONE

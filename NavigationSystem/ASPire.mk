###############################################################################
#
# ASPire Makefile
#
###############################################################################


###############################################################################
# Files
###############################################################################

# SRC += main_ASPire.cpp
# OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


SRC = main_ASPire.cpp
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

HARDWARE_SERVICES_SRC = HardwareServices/i2ccontroller/I2CController.cpp \
							   HardwareServices/CAN_Services/CANPGNReceiver.cpp HardwareServices/CAN_Services/CANService.cpp \
							   HardwareServices/CAN_Services/mcp2515.cpp HardwareServices/CAN_Services/MsgFunctions.cpp \
							   HardwareServices/CAN_Services/CANFrameReceiver.cpp

HARDWARE_NODES_SRC   = Nodes/HMC6343Node.cpp Nodes/GPSDNode.cpp Nodes/ActuatorNodeASPire.cpp \
                            Nodes/CANFeedbackReceiver.cpp Nodes/CANWindsensorNode.cpp

CORE_SRC             = 	Nodes/WaypointMgrNode.cpp Nodes/LowLevelControllerNodeASPire.cpp \
										Nodes/StateEstimationNode.cpp Nodes/WindStateNode.cpp \
										$(MESSAGE_BUS_SRC) $(NETWORK_SRC) $(SYSTEM_SERVICES_SRC) $(MATH_SRC)
ifeq ($(USE_LNM),1)
SRC 											+= $(MAIN_LNM_SRC) $(CORE_SRC) $(LNM_SRC)
else
SRC 											+= $(CORE_SRC) $(LINE_FOLLOW_SRC) $(HTTP_SYNC_SRC)
endif

ifeq ($(USE_SIM),1)
SRC 											+= $(SIMULATOR_SRC)
else
SRC 											+= $(HARDWARE_NODES_SRC) $(HARDWARE_SERVICES_SRC) $(XBEE_NETWORK_SRC)
endif
###############################################################################
# Rules
###############################################################################

all: $(EXECUTABLE) stats

# Link and build
$(EXECUTABLE): $(OBJECTS)
		rm -f $(OBJECT_FILE)
		@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
		@echo Linking object files
		$(CXX) $(LDFLAGS) @$(OBJECT_FILE) ./libwiringPi.so -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
		@mkdir -p $(dir $@)
		@echo Compiling CPP File: $@

		@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(EXECUTABLE)
		@echo Final executable size:
		$(SIZE) $(EXECUTABLE)

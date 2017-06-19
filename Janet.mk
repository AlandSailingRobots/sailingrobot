###############################################################################
#
# Janet Makefile
#
###############################################################################


###############################################################################
# Files
###############################################################################

# SRC += main_janet.cpp
# OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

###############################################################################
# Rules
###############################################################################

SRC = main_janet.cpp
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

HARDWARE_SERVICES_SRC = HardwareServices/MaestroController/MaestroController.cpp \
										HardwareServices/i2ccontroller/I2CController.cpp

HARDWARE_NODES_SRC   = Nodes/CV7Node.cpp Nodes/HMC6343Node.cpp Nodes/GPSDNode.cpp \
                    Nodes/ActuatorNode.cpp Nodes/ArduinoNode.cpp


CORE_SRC             = 	Nodes/WaypointMgrNode.cpp Nodes/StateEstimationNode.cpp \
										Nodes/LowLevelControllerNodeJanet.cpp Nodes/WindStateNode.cpp \
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

all: $(EXECUTABLE) stats

#$(EXECUTABLE): $(OBJECTS)

# Link and build
$(EXECUTABLE): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) ./libwiringPi.so -Wl,-rpath=./ -o $@ $(LIBS) #./libwiringPi.so

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@

	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(EXECUTABLE)
	@echo Final executable size:
	$(SIZE) $(EXECUTABLE)

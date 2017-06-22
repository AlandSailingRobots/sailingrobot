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

# SRC = main_janet.cpp
MAIN = main_janet.cpp
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

HARDWARE_SERVICES_SRC = Hardwares/MaestroController/MaestroController.cpp \
										Hardwares/i2ccontroller/I2CController.cpp

HARDWARE_NODES_SRC   = Hardwares/CV7Node.cpp Hardwares/HMC6343Node.cpp Hardwares/GPSDNode.cpp \
                    Hardwares/ActuatorNode.cpp Hardwares/ArduinoNode.cpp


CORE_SRC             = 	Navigation/WaypointMgrNode.cpp WorldState/StateEstimationNode.cpp \
										LowLevelControllers/LowLevelControllerNodeJanet.cpp WorldState/WindStateNode.cpp \
										WorldState/VesselStateNode.cpp \
										$(MESSAGE_BUS_SRC) $(NETWORK_SRC) $(SYSTEM_SERVICES_SRC) $(MATH_SRC)

ifeq ($(USE_LNM),1)
SRC 											= $(MAIN) $(CORE_SRC) $(LNM_SRC)
else
SRC 											= $(MAIN) $(CORE_SRC) $(LINE_FOLLOW_SRC) $(HTTP_SYNC_SRC)
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
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS) #./libwiringPi.so

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@

	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(EXECUTABLE)
	@echo Final executable size:
	$(SIZE) $(EXECUTABLE)

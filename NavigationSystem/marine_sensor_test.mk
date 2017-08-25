###############################################################################
#
# Makefile for building the integration tests for the marine sensors.
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
MAIN_MARINE_SENSOR_TESTS 	= Tests/IntegrationTests/MarineSensorIntegrationTest.cpp

SRC 					= $(MAIN_MARINE_SENSOR_TESTS) $(CORE_SRC) $(HW_SERVICES_ALL_SRC) \
							$(CAN_SERVICES_SRC) $(HW_NODES_ALL_SRC) $(HW_NODES_ASPIRE_SRC)


# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################

all: $(MARINE_SENSOR_TEST_EXCE) stats

# Link and build
$(MARINE_SENSOR_TEST_EXCE): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(MARINE_SENSOR_TEST_EXCE)
	@echo Final executable size:
	$(SIZE) $(MARINE_SENSOR_TEST_EXCE)

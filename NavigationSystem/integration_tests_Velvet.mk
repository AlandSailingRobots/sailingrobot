###############################################################################
#
# Makefile for building the integration tests for Velvet
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
#MAIN_INTEGRATION_TESTS 	= Tests/IntegrationTests/SensorVelvetIntegrationTest.cpp
MAIN_INTEGRATION_TESTS 	= Tests/IntegrationTests/SensorVelvetIntegrationTest.cpp

SRC 					= $(MAIN_INTEGRATION_TESTS) $(CORE_SRC) $(HW_SERVICES_ALL_SRC) \
							$(HW_NODES_ALL_SRC) $(HW_NODES_VELVET_SRC) $(HW_SERVICES_VELVET_SRC)

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################

all: $(INTEGRATION_TEST_EXEC_VELVET) stats

# Link and build
$(INTEGRATION_TEST_EXEC_VELVET): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(INTEGRATION_TEST_EXEC_VELVET)
	@echo Final executable size:
	$(SIZE) $(INTEGRATION_TEST_EXEC_VELVET)

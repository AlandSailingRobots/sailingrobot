###############################################################################
#
# Makefile for building the integration tests for the AIS.
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
CANAIS_INTEGRATION_TEST = Tests/IntegrationTests/CANAISTest.cpp

SRC = $(CORE_SRC) $(HW_NODES_ASPIRE_SRC) $(HARDWARE_SERVICES_SRC) $(CAN_SERVICES_SRC) $(CANAIS_INTEGRATION_TEST) $(COLLIDABLE_MGR_SRC)

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################


all: $(AIS_TEST_EXEC) stats

# Link and build
$(AIS_TEST_EXEC): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@

	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(AIS_TEST_EXEC)
	@echo Final executable size:
	$(SIZE) $(AIS_TEST_EXEC)

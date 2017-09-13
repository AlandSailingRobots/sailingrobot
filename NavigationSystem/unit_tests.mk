###############################################################################
#
# Makefile for building the unit tests
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
SRC         = $(CORE_SRC) $(LNM_SRC) $(COLLIDABLE_MGR_SRC) $(LINE_FOLLOW_SRC) $(SIMULATOR_SRC) \
				$(HW_SERVICES_ALL_SRC) $(CAN_SERVICES_SRC) $(HW_SERVICES_JANET_SRC) \
				$(HW_NODES_ALL_SRC) $(HW_NODES_ASPIRE_SRC) $(HW_NODES_JANET_SRC) \
				$(XBEE_NETWORK_SRC)


# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################


all: $(UNIT_TEST_EXEC) $(UNIT_TEST_HW_EXEC) stats

# Link and build
$(UNIT_TEST_EXEC): $(OBJECTS) Tests/runner.o
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) Tests/runner.o @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

$(UNIT_TEST_HW_EXEC): $(OBJECTS) Tests/runner.o
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) Tests/runnerHardware.o @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(UNIT_TEST_EXEC) $(UNIT_TEST_HW_EXEC)
	@echo Final executable size:
	$(SIZE) $(UNIT_TEST_EXEC)
	$(SIZE) $(UNIT_TEST_HW_EXEC)

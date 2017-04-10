###############################################################################
#
# Tests Makefile
#
###############################################################################


###############################################################################
# Files
###############################################################################

#UNIT_TEST_SRC		= unit-tests/runner.cpp
#HARDWARE_TESTS_SRC  = unit-tests/runnerHardware.cpp

SRC         = $(CORE_SRC) $(LINE_FOLLOW_SRC) $(HTTP_SYNC_SRC) $(SIMULATOR_SRC) \
			$(HARDWARE_NODES_SRC) $(HARDWARE_SERVICES_SRC) $(XBEE_NETWORK_SRC)

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################


all: $(UNIT_TEST_EXEC) $(HARDWARE_TEST_EXEC) stats

# Link and build
$(UNIT_TEST_EXEC): $(OBJECTS) Tests/runner.o
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) Tests/runner.o @$(OBJECT_FILE) ./libwiringPi.so -Wl,-rpath=./ -o $@ $(LIBS)

$(HARDWARE_TEST_EXEC): $(OBJECTS) Tests/runner.o
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) Tests/runnerHardware.o @$(OBJECT_FILE) ./libwiringPi.so -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@

	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(UNIT_TEST_EXEC) $(HARDWARE_TEST_EXEC)
	@echo Final executable size:
	$(SIZE) $(UNIT_TEST_EXEC)
	$(SIZE) $(HARDWARE_TEST_EXEC)
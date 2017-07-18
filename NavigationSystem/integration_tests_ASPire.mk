###############################################################################
#
# Makefile for the integration test for ASPire
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
MAIN_INTEGRATION_TESTS 	= Tests/IntegrationTests/ArduinoIntegrationTest.cpp

SRC 					= $(MAIN_INTEGRATION_TESTS) $(CORE_SRC) $(HW_SERVICES_ALL_SRC) \
							$(CAN_SERVICES_SRC) $(HW_NODES_ALL_SRC) $(HW_NODES_ASPIRE_SRC)


# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################

all: $(INTEGRATION_TEST_EXEC_ASPIRE) stats

# Link and build
$(INTEGRATION_TEST_EXEC_ASPIRE): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(INTEGRATION_TEST_EXEC_ASPIRE)
	@echo Final executable size:
	$(SIZE) $(INTEGRATION_TEST_EXEC_ASPIRE)

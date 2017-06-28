###############################################################################
#
# Makefile for the integration test for ASPire
#
###############################################################################


###############################################################################
# Files
###############################################################################

CORE_SRC							+= $(CORE_ASPIRE)

HARDWARE_NODES_SRC 		+= $(HARDWARE_NODES_ASPIRE)

HARDWARE_SERVICES_SRC += $(CAN_HARDWARE_SRC)

SRC 									= $(CORE_SRC) $(HARDWARE_NODES_SRC) $(HARDWARE_SERVICES_SRC) \
											$(INTEGRATION_TEST)

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

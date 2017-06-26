###############################################################################
#
# ASPire Makefile
#
###############################################################################


###############################################################################
# Build the SRC varibale
###############################################################################

ifeq ($(USE_LNM),1)
SRC 											= $(MAIN_ASPIRE) $(CORE_SRC_ASPIRE) $(LNM_SRC)
else
SRC 											= $(MAIN_ASPIRE) $(CORE_SRC_ASPIRE) $(LINE_FOLLOW_SRC) $(HTTP_SYNC_SRC)
endif

ifeq ($(USE_SIM),1)
SRC 											+= $(SIMULATOR_SRC)
else
SRC 											+= $(HARDWARE_NODES_SRC_ASPIRE) $(HARDWARE_SERVICES_SRC_ASPIRE) $(XBEE_NETWORK_SRC)
endif

OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

###############################################################################
# Rules
###############################################################################

all: $(EXECUTABLE) stats

# Link and build
$(EXECUTABLE): $(OBJECTS)
		rm -f $(OBJECT_FILE)
		@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
		@echo Linking object files
		$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
		@mkdir -p $(dir $@)
		@echo Compiling CPP File: $@

		@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(EXECUTABLE)
		@echo Final executable size:
		$(SIZE) $(EXECUTABLE)

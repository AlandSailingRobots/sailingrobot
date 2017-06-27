###############################################################################
#
# ASPire Makefile
#
###############################################################################


###############################################################################
# Build the SRC varibale
###############################################################################

CORE_SRC									+= $(CORE_ASPIRE)

HARDWARE_SERVICES_SRC			+= $(CAN_HARDWARE_SRC)

HARDWARE_NODES_SRC				+= $(HARDWARE_NODES_ASPIRE)

ifeq ($(USE_LNM),1)
SRC 											= $(LNM_SRC)
else
SRC 											= $(LINE_FOLLOW_SRC)
endif

ifeq ($(USE_SIM),1)
SRC 											+= $(SIMULATOR_SRC)
else
SRC 											+= $(HARDWARE_NODES_SRC) $(HARDWARE_SERVICES_SRC) $(XBEE_NETWORK_SRC)
endif

SRC												+= $(CORE_SRC) $(HTTP_SYNC_SRC) $(MAIN_ASPIRE)

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

###############################################################################
#
# Makefile for building the control system for ASPire
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
SRC						= main_ASPire.cpp $(CORE_SRC) $(COLLIDABLE_MGR_SRC)

ifeq ($(USE_LNM),1)
SRC 					+= $(LNM_SRC)
else
SRC 					+= $(LINE_FOLLOW_SRC)
endif

ifeq ($(USE_SIM),1)
SRC 					+= $(SIMULATOR_SRC)
else
SRC 					+= $(HW_SERVICES_ALL_SRC) $(CAN_SERVICES_SRC) $(HW_NODES_ALL_SRC) \
							$(HW_NODES_ASPIRE_SRC)
endif


# Object files
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

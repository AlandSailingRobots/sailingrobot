###############################################################################
#
# Local Navigation Development Makefile
#
###############################################################################


###############################################################################
# Files
###############################################################################


SRC         = $(MAIN_LNM_SRC) $(CORE_SRC) $(LNM_SRC)

ifeq ($(USE_SIM),1)

SRC         += $(SIMULATOR_SRC)

else

SRC         += $(HARDWARE_NODES_SRC) $(HARDWARE_SERVICES_SRC)

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
###############################################################################
#
# Makefile for building the obstacle detection integration test.
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

LIBS += $(shell pkg-config --cflags --libs opencv)

# Source files
OBSTACLE_DETECTION_INTEGRATION_TEST = Tests/IntegrationTests/ObstacleDetectionTest.cpp

SRC = $(CORE_SRC) $(HW_NODES_ALL_SRC) $(HW_SERVICES_ALL_SRC) $(HW_NODES_CAMERA) \
	$(COLLIDABLE_MGR_SRC) $(OBSTACLE_DETECTION_INTEGRATION_TEST)

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################


all: $(OBSTACLE_DETECTION_TEST_EXEC) stats

# Link and build
$(OBSTACLE_DETECTION_TEST_EXEC): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@

	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(OBSTACLE_DETECTION_TEST_EXEC)
	@echo Final executable size:
	$(SIZE) $(OBSTACLE_DETECTION_TEST_EXEC)

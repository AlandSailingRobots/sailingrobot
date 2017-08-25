###############################################################################
#
# Makefile for building the integration tests for the HTTPSync with the server.
#
# This makefile cannot be run directly. Use the master makefile instead.
#
###############################################################################


###############################################################################
# Files
###############################################################################

# Source files
HTTP_SYNC_TEST_MAIN	 	= Tests/IntegrationTests/HTTPSyncTest.cpp

SRC 					= $(CORE_SRC) $(HTTP_SYNC_TEST_MAIN)

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))


###############################################################################
# Rules
###############################################################################

all: $(HTTP_SYNC_TEST_EXEC) stats

# Link and build
$(HTTP_SYNC_TEST_EXEC): $(OBJECTS)
	rm -f $(OBJECT_FILE)
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	@echo Linking object files
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) -Wl,-rpath=./ -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC_DIR) -o ./$@ $< $(DEFINES) $(LIBS)

stats:$(HTTP_SYNC_TEST_EXEC)
	@echo Final executable size:
	$(SIZE) $(HTTP_SYNC_TEST_EXEC)

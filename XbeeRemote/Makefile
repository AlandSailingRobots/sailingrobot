#####################################################
#                                                   #
#    Aland Sailing Robot                            #
#    ===========================================    #
#    xBeeRelay                                		#
#    -------------------------------------------    #
#                                                   #
#####################################################



#####################################################
# Files
#####################################################


BUILD_DR 		= ../build/XbeeRemote

SRC 			= main.cpp XbeeRemote.cpp UDPRelay.cpp

ifeq ($(TOOLCHAIN),win)

TARGET			= ./XbeeRemote.exe
SRC				+= WindowsSerialDataLink.cpp

else

TARGET			= ./XbeeRemote.run
SRC				+= udpclient.cpp
OBJECTS			= ../build/Network/LinuxSerialDataLink.o

WIRING_PI 		= libwiringPi.so
MOVE_LIB		= @mv ../$(WIRING_PI) $(WIRING_PI)

endif


OBJECTS 		+= $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))
OBJECTS 		+= ../build/Network/DataLink.o ../build/Network/XbeePacketNetwork.o ../build/utility/SysClock.o ../build/SystemServices/Logger.o ../build/Messages/MessageSerialiser.o ../build/Messages/MessageDeserialiser.o

INCLUDE			= -I../


#####################################################
# Rules
#####################################################


all : $(TARGET)
	 
$(TARGET): $(OBJECTS)
	$(MOVE_LIB)
	@$(CXX) $(CPPFLAGS) $(OBJECTS) -Wl,-rpath=./,-rpath=../ $(WIRING_PI) -o $(TARGET) $(LIBS)
	@echo Built $(TARGET)
	 
# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	$(CXX) -c $(CPPFLAGS) $(INC) $(INCLUDE) -o ./$@ $< -DTOOLCHAIN=$(TOOLCHAIN) $(LIBS)

clean :
	rm -f ./*.o
	rm xbeerelay

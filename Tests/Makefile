

USE_DEV_LNM = 0
#####################################################################
# Files

# Source files
TEST_SRC := $(wildcard ../../tests/unit-tests/*.h)


ifeq ($(USE_DEV_LNM), 1)


UNIT_TEST_SRCS 			= MessageCoreSuite.h MessageSuite.h UtilitySuite.h HTTPSyncSuite.h WaypointNodeSuite.h ASRCourseBallotSuite.h ASRArbiterSuite.h

else

UNIT_TEST_SRCS 			= MessageCoreSuite.h MessageSuite.h UtilitySuite.h HTTPSyncSuite.h WaypointNodeSuite.h LineFollowSuite.h


HARDWARE_TEST_SRCS    	= HardwareXbeeSuite.h HardwareCompassSuite.h HardwareCV7Suite.h HardwareGPSDSuite.h MaestroControllerSuite.h ActuatorSailNodeSuite.h \
						ActuatorRudderNodeSuite.h
						# source file of the tests of the I2C branch :
						# ArduinoI2CSuite.h I2CControllerSuite.h HardwareWiringPiSuite.h

TEST_MOCK_SRCS 			= MockNode.h

endif


UNIT_TESTS = $(addprefix unit-tests/, $(UNIT_TEST_SRCS))
HARDWARE_TESTS = $(addprefix unit-tests/, $(HARDWARE_TEST_SRCS))
TEST_MOCKS = $(addprefix unit-tests/TestMocks, $(TEST_MOCK_SRCS))



#####################################################################
# Compile Tools

TESTGEN_FLAGS = --error-printer --have-eh

# Tools
CXX = g++ -c -Wall -W -I./unit-tests -I./cxxtest -I../ -std=c++14
TEST_GEN = python cxxtest/bin/cxxtestgen

#####################################################################
# Rules

.phony: test-gen clean

all: test-gen

test-gen: clean
	$(TEST_GEN) $(TESTGEN_FLAGS) -o runner.cpp $(UNIT_TESTS) $(TEST_MOCKS)
	$(CXX) runner.cpp -o runner.o
	$(TEST_GEN) $(TESTGEN_FLAGS) -o runnerHardware.cpp $(HARDWARE_TESTS) $(TEST_MOCKS)
	$(CXX) runnerHardware.cpp -o runnerHardware.o

test-gen-lnm: clean
	$(TEST_GEN) $(TESTGEN_FLAGS) -o runner.cpp $(UNIT_TESTS) $(TEST_MOCKS)
	$(CXX) runner.cpp -o runner.o

clean:
	-@rm -f runner.cpp
	-@rm -f runner.o
	-@rm -f runnerHardware.cpp
	-@rm -f runnerHardware.o

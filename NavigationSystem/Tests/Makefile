#############################################################################################
#
# Makefile for building the unit tests source and object files
#
# This makefile does not creat the executable file. Use the master makefile instead.
#
#############################################################################################



#############################################################################################
# Tools
#############################################################################################

TESTGEN_FLAGS = --error-printer --have-eh
TEST_GEN = python cxxtest/bin/cxxtestgen
CXX = g++ -c -Wall -W -I./unit-tests -I./cxxtest -I../ -std=c++14


#############################################################################################
# Files
#############################################################################################

# Header files
UNIT_TEST_SRCS 		= MessageCoreSuite.h MessageSuite.h UtilitySuite.h HTTPSyncSuite.h \
						WaypointNodeSuite.h LineFollowSuite.h CANWindsensorNodeSuite.h \
					  	WindStateNodeSuite.h StateEstimationNodeSuite.h WingsailControlSuite.h \
					  	CourseRegulatorSuite.h CANMessageSuite.h DBLoggerNodeSuite.h \
					  	LowLevelControllerNodeJanetSuite.h LowLevelControllersFunctionsTestSuite.h \
					  	ASRCourseBallotSuite.h AISProcSuite.h CanNodesSuite.h
					  	# ASRArbiterSuite.h // NOTE - Maël: This unit test suite is the source of a building error.

HARDWARE_TEST_SRCS	= HardwareXbeeSuite.h HardwareCompassSuite.h HardwareCV7Suite.h HardwareGPSDSuite.h \
						MaestroControllerSuite.h ActuatorSailNodeSuite.h ActuatorRudderNodeSuite.h \
						CANServiceSuite.h ActuatorNodeASPireSuite.h CANArduinoNodeSuite.h \
						I2CControllerSuite.h HardwareWiringPiSuite.h

TEST_MOCK_SRCS 		= MockNode.h


UNIT_TESTS 			= $(addprefix unit-tests/, $(UNIT_TEST_SRCS))
HARDWARE_TESTS 		= $(addprefix unit-tests/, $(HARDWARE_TEST_SRCS))
TEST_MOCKS 			= $(addprefix unit-tests/TestMocks, $(TEST_MOCK_SRCS))


#############################################################################################
# Rules
#############################################################################################

.phony: test-gen clean

all: test-gen

test-gen: clean
	$(TEST_GEN) $(TESTGEN_FLAGS) -o runner.cpp $(UNIT_TESTS) $(TEST_MOCKS)
	$(CXX) runner.cpp -o runner.o
	$(TEST_GEN) $(TESTGEN_FLAGS) -o runnerHardware.cpp $(HARDWARE_TESTS) $(TEST_MOCKS)
	$(CXX) runnerHardware.cpp -o runnerHardware.o

clean:
	-@rm -f runner.cpp
	-@rm -f runner.o
	-@rm -f runnerHardware.cpp
	-@rm -f runnerHardware.o

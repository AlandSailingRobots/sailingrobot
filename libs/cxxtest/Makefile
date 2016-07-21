####################################################################
#
# Added by Jordan Less'ard-Springett
# https://github.com/JSpringett
#
####################################################################


#####################################################################
# Files

BUILD_DIR = ../../build

# Source files
TEST_SRC := $(wildcard ../../tests/unit-tests/*.h)
OBJECTS = @../../build/objects.tmp

#####################################################################
# Compile Tools

TESTGEN_FLAGS = --error-printer --have-eh

# Tools
CXX = g++ -c -Wall -W -I. -I../../ -I../../../tests/uni-tests -std=c++11
TEST_GEN = python bin/cxxtestgen

#####################################################################
# Rules

.phony: test-gen clean

all: test-gen

test-gen: clean
	$(TEST_GEN) $(TESTGEN_FLAGS) -o runner.cpp $(TEST_SRC)
	$(CXX) runner.cpp -o runner.o

clean:
	rm -f runner.cpp
	rm -f runner.o

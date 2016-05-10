#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    PressureSensor
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS = -lpthread -lwiringPi -lrt -I$(SAILINGROBOTS_HOME)

SOURCES_MOCK = MockPressureSensor.cpp
HEADERS_MOCK = MockPressureSensor.h
FILE_MOCK = MockPressureSensor.o

SOURCES_COMP = PressureSensor.cpp
HEADERS_COMP = PressureSensor.h
FILE_COMP = PressureSensor.o

SOURCES_HMC = PTMN_STS.cpp
HEADERS_HMC = PTMN_STS.h
FILE_HMC = PTMN_STS.o

FILES = $(FILE_HMC) $(FILE_COMP) $(FILE_MOCK)

SOURCES = $(SOURCES_COMP) $(SOURCES_HMC) $(SOURCES_UTIL) $(SOURCES_MOCK)
HEADERS = $(HEADERS_COMP) $(HEADERS_HMC) $(HEADERS_UTIL) $(HEADERS_MOCK)

all: $(FILES)

$(FILE_MOCK): $(SOURCES_MOCK) $(HEADERS_MOCK)
	$(CC) $(SOURCES_MOCK) $(FLAGS) $(LIBS) -c -o $(FILE_MOCK)

$(FILE_COMP): $(SOURCES_COMP) $(HEADERS_COMP)
	$(CC) $(SOURCES_COMP) $(FLAGS) $(LIBS) -c -o $(FILE_COMP)

$(FILE_HMC): $(SOURCES_HMC) $(HEADERS_HMC)
	$(CC) $(SOURCES_HMC) $(FLAGS) $(LIBS) -c -o $(FILE_HMC)


clean:
	rm -f $(FILES)

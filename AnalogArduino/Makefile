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

SOURCES_MOCK = MockAnalogArduino.cpp
HEADERS_MOCK = MockAnalogArduino.h
FILE_MOCK = MockAnalogArduino.o

SOURCES_COMP = AnalogArduino.cpp
HEADERS_COMP = AnalogArduino.h
FILE_COMP = AnalogArduino.o

SOURCES_HMC = AR_UNO.cpp
HEADERS_HMC = AR_UNO.h
FILE_HMC = AR_UNO.o

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

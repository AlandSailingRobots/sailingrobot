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



SOURCES_BEH = RoutingBehaviour.cpp
HEADERS_BEH = RoutingBehaviour.h
FILE_BEH = RoutingBehaviour.o

SOURCES_WAY = WaypointBehaviour.cpp
HEADERS_WAY = WaypointBehaviour.h
FILE_WAY = WaypointBehaviour.o

SOURCES_LINE = LineFollowBehaviour.cpp
HEADERS_LINE = LineFollowBehaviour.h
FILE_LINE = LineFollowBehaviour.o


FILES = $(FILE_BEH) $(FILE_WAY) $(FILE_LINE)

SOURCES = $(SOURCES_BEH) $(SOURCES_WAY) $(SOURCES_LINE)
HEADERS = $(HEADERS_BEH) $(HEADERS_WAY) $(HEADERS_LINE)

all: $(FILES)

$(FILE_BEH): $(SOURCES_BEH) $(HEADERS_BEH)
	$(CC) $(SOURCES_BEH) $(FLAGS) $(LIBS) -c -o $(FILE_BEH)

$(FILE_WAY): $(SOURCES_WAY) $(HEADERS_WAY)
	$(CC) $(SOURCES_WAY) $(FLAGS) $(LIBS) -c -o $(FILE_WAY)

$(FILE_LINE) : $(SOURCES_LINE) $(HEADERS_LINE)
	$(CC) $(SOURCES_LINE) $(FLAGS) $(LIBS) -c -o $(FILE_LINE)

clean:
	rm -f $(FILES)

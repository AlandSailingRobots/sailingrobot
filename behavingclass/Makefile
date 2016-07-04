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



SOURCES_BEH = behavingclass.cpp
HEADERS_BEH = behavingclass.h
FILE_BEH = behavingclass.o

SOURCES_WAY = waypointBehave.cpp
HEADERS_WAY = waypointBehave.h
FILE_WAY = waypointBehave.o

SOURCES_LINE = LineFollowBehave.cpp
HEADERS_LINE = LineFollowBehave.h
FILE_LINE = LineFollowBehave.o

SOURCES_COLL = CollisionAvoidanceBehave.cpp
HEADERS_COLL = CollisionAvoidanceBehave.h
FILE_COLL = CollisionAvoidanceBehave.o

FILES = $(FILE_BEH) $(FILE_WAY) $(FILE_LINE) $(FILE_COLL)

SOURCES = $(SOURCES_BEH) $(SOURCES_WAY) $(SOURCES_LINE) $(SOURCES_COLL)
HEADERS = $(HEADERS_BEH) $(HEADERS_WAY) $(HEADERS_LINE) $(HEADERS_COLL)

all: $(FILES)

$(FILE_BEH): $(SOURCES_BEH) $(HEADERS_BEH)
	$(CC) $(SOURCES_BEH) $(FLAGS) $(LIBS) -c -o $(FILE_BEH)

$(FILE_WAY): $(SOURCES_WAY) $(HEADERS_WAY)
	$(CC) $(SOURCES_WAY) $(FLAGS) $(LIBS) -c -o $(FILE_WAY)

$(FILE_LINE) : $(SOURCES_LINE) $(HEADERS_LINE)
	$(CC) $(SOURCES_LINE) $(FLAGS) $(LIBS) -c -o $(FILE_LINE)

$(FILE_COLL) : $(SOURCES_COLL) $(HEADERS_COLL)
	$(CC) $(SOURCES_COLL) $(FLAGS) $(LIBS) -c -o $(FILE_COLL)

clean:
	rm -f $(FILES)

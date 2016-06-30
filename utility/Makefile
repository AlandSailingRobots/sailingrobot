#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    utility
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14 
LIBS = -I$(SAILINGROBOTS_HOME)

SOURCES = Utility.cpp 
HEADERS = Utility.h 
FILE = Utility.o

SOURCES_REALPOS = RealPosition.cpp 
HEADERS_REALPOS = RealPosition.h 
FILE_REALPOS = RealPosition.o

SOURCES_MOCKPOS = MockPosition.cpp
HEADERS_MOCKPOS = MockPosition.h 
FILE_MOCKPOS = MockPosition.o

SOURCES_POS = Position.cpp
HEADERS_POS = Position.h
FILE_POS = Position.o

SOURCES_TIMER = Timer.cpp
FILE_TIMER = Timer.o

OBJ = $(FILE) $(FILE_TIMER) $(FILE_REALPOS) $(FILE_MOCKPOS) $(FILE_POS)

all: $(OBJ)

$(FILE): $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)

$(FILE_TIMER): $(SOURCES_TIMER) Timer.h
	$(CC) $(SOURCES_TIMER) $(FLAGS) $(LIBS) -c -o $(FILE_TIMER)

$(FILE_REALPOS) : $(SOURCES_REALPOS) $(HEADERS_REALPOS)	
	$(CC) $(SOURCES_REALPOS) $(FLAGS) $(LIBS) -c -o $(FILE_REALPOS)
	
$(FILE_MOCKPOS) : $(SOURCES_MOCKPOS) $(HEADERS_MOCKPOS)	
	$(CC) $(SOURCES_MOCKPOS) $(FLAGS) $(LIBS) -c -o $(FILE_MOCKPOS)
	
$(FILE_POS) : $(SOURCES_POS) $(HEADERS_POS)	
	$(CC) $(SOURCES_POS) $(FLAGS) $(LIBS) -c -o $(FILE_POS)
	
clean:
	rm -f $(OBJ)

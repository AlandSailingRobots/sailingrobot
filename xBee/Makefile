#####################################################
#                                                   #
#    Aland Sailing Robot                            #
#    ===========================================    #
#    sailingrobot                                   #
#    -------------------------------------------    #
#                                                   #
#####################################################

CC = g++ 
FLAGS = -g -Wall -pedantic -Werror -std=c++14 -I$(SAILINGROBOTS_HOME)

LIBS = -lsqlite3 -lgps -lrt -lwiringPi -lcurl

SOURCES = xBee.cpp 
HEADERS = xBee.h 

SOURCES_SYNC = xBeeSync.cpp
HEADERS_SYNC = xBeeSync.h

FILE = xBee.o 
FILE_SYNC = xBeeSync.o


all : $(FILE) $(FILE_SYNC)
	

$(FILE) : $(SOURCES) $(HEADERS)
	 $(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)

$(FILE_SYNC) : $(SOURCES_SYNC) $(HEADERS_SYNC)
	 $(CC) $(SOURCES_SYNC) $(FLAGS) $(LIBS) -c -o $(FILE_SYNC)

clean :
	rm -f ./*.o

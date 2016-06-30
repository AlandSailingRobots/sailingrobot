#####################################################
#                                                   #
#    Aland Sailing Robot                            #
#    ===========================================    #
#    xBeeRelay                                		#
#    -------------------------------------------    #
#                                                   #
#####################################################

CC = g++ 
FLAGS = -Wall -pedantic -Werror -std=c++14 -I$(SAILINGROBOTS_HOME)

LIBS = -lsqlite3 -lgps -lrt -lwiringPi -lcurl

SOURCES_RELAY 	= main.cpp xBeeRelay.cpp 
HEADERS_RELAY 	= xBeeRelay.h 
OBJ_RELAY 		= $(SAILINGROBOTS_HOME)/xBee/xBee.o 

FILE_RELAY = xbeerelay


all : $(FILE_RELAY)
	
$(FILE_RELAY) : $(SOURCES_RELAY) $(HEADERS_RELAY)
	 $(CC) $(SOURCES_RELAY) $(OBJ_RELAY) $(FLAGS) $(LIBS) -o $(FILE_RELAY)

clean :
	rm -f ./*.o
	rm xbeerelay

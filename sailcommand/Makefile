#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailcommand
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS =

SOURCES = SailCommand.cpp
HEADERS = SailCommand.h
FILE = SailCommand.o



all : $(FILE)
$(FILE) : $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)


clean :
	rm -f $(FILE)

#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    ruddercommand
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS =

SOURCES = RudderCommand.cpp
HEADERS = RudderCommand.h
FILE = RudderCommand.o



all : $(FILE)
$(FILE) : $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)

clean :
	rm -f $(FILE)
	rm -f *.gcda
	rm -f *.gcno

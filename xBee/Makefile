#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailingrobot
#    -------------------------------------------
#
#######################################################

CC = g++ 
FLAGS = -Wall -pedantic -Werror -std=c++14 
LIBS = -lsqlite3 -lgps -lrt -lwiringPi -lcurl
SOURCES = xBee.cpp
HEADERS = xBee.h
FILE = xBee.o



all : $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)


clean :
	rm -f ./xBee.o

gitclean :
	rm -f ./*.cpp
	rm -f ./*.h




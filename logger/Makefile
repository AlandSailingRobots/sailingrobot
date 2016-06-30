#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    logger
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -pthread -std=c++14
LIBS =  -lboost_system -lboost_log_setup -lboost_log -lboost_date_time -lboost_thread -lrt -lboost_filesystem

SOURCES = Logger.cpp
HEADERS = Logger.h
FILE = Logger.o


all : $(FILE)
$(FILE) : $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)

clean :
	rm -f $(FILE)

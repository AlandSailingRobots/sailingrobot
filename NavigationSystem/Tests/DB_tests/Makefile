#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    testsuit - dbhandler
#    -------------------------------------------
#
#######################################################


CC = g++
# for gcov:
#FLAGS = -g -std=c++14 -Wall -pedantic -Werror --coverage
FLAGS = -g -std=c++14 -Wall -pedantic #-Werror
LIBS = -I$(SAILINGROBOTS_HOME) -I$(SAILINGROBOTS_HOME)/tests -lsqlite3 -lpthread
LIBS_BOOST = -lboost_system -lboost_log -lboost_thread -I$(SAILINGROBOTS_HOME)

HEADERS = ../catch.hpp
SOURCES = main.cpp \
		  test_dbhandler.cpp
MAKE = make

DB = $(SAILINGROBOTS_HOME)/dbhandler/DBHandler.o
LOGGER = $(SAILINGROBOTS_HOME)/logger/Logger.o

OBJECTS = $(DB) $(LOGGER)

EXE = db_tests

all : $(EXE)

clean:
	rm -f $(EXE) testdb.db

$(EXE): $(SOURCES) $(HEADERS) $(OBJECTS)
	$(CC) $(SOURCES) $(HEADERS) $(OBJECTS) $(FLAGS) $(LIBS) $(LIBS_BOOST) -o $(EXE)

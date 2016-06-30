#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    dbhandler
#    -------------------------------------------
#
#######################################################

# for gcov:
#CC1 = g++ -c -g -O0 --coverage

CC = g++
FLAGS = -g -pedantic -Werror -std=c++14
LIBS = -lsqlite3 -I$(SAILINGROBOTS_HOME)

SOURCES_DB = DBHandler.cpp
HEADERS_DB = DBHandler.h
FILE_DB = DBHandler.o


HEADERS = $(HEADERS_DB)
SOURCES = $(SOURCES_DB)

all:  $(FILE_DB)

# insert CC1 instead of CC when using gcov:
$(FILE_DB): $(SOURCES_DB) $(HEADERS_DB)
	$(CC) $(SOURCES_DB) $(FLAGS) $(LIBS) -c -o $(FILE_DB)


clean:
	rm -f $(FILE_DB) 

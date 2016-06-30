CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS = -lpthread -lwiringPi -lrt -I$(SAILINGROBOTS_HOME)

SOURCES = ExternalCommand.cpp
HEADERS = ExternalCommand.h
FILE = ExternalCommand.o

SOURCES_SYS = SystemState.cpp
HEADERS_SYS = SystemState.h
FILE_SYS = SystemState.o

SOURCES_RAII = ThreadRAII.cpp
HEADERS_RAII = ThreadRAII.h
FILE_RAII = ThreadRAII.o

all : $(FILE) $(FILE_SYS) $(FILE_RAII)

$(FILE) : $(SOURCES) $(HEADERS)
	$(CC) $(SOURCES) $(FLAGS) $(LIBS) -c -o $(FILE)

$(FILE_SYS) : $(SOURCES_SYS) $(HEADERS_SYS)
	$(CC) $(SOURCES_SYS) $(FLAGS) $(LIBS) -c -o $(FILE_SYS)

$(FILE_RAII) : $(SOURCES_RAII) $(HEADERS_RAII)
	$(CC) $(SOURCES_RAII) $(FLAGS) $(LIBS) -c -o $(FILE_RAII)

clean :
	rm -f $(FILE) $(FILE_SYS) $(FILE_RAII)

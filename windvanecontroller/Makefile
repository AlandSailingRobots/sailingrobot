#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    windvanecontroller
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS = -I$(SAILINGROBOTS_HOME)

SOURCES_WPR = WindVaneController.cpp
HEADERS_WPR = WindVaneController.h
FILE_WPR = WindVaneController.o

#SOURCES_CMD = Commands.cpp
#HEADERS_CMD = Commands.h
#FILE_CMD = Commands.o

#SOURCES_TA = TackAngle.cpp
#HEADERS_TA = TackAngle.h
#FILE_TA = TackAngle.o

all : $(FILE_WPR) $(FILE_CMD) $(FILE_TA)

$(FILE_WPR) : $(SOURCES_WPR) $(HEADERS_WPR)
	$(CC) $(SOURCES_WPR) $(FLAGS) $(LIBS) -c -o $(FILE_WPR)

$(FILE_CMD) : $(SOURCES_CMD) $(HEADERS_CMD)
	$(CC) $(SOURCES_CMD) $(FLAGS) $(LIBS) -c -o $(FILE_CMD)

$(FILE_TA) : $(SOURCES_TA) $(HEADERS_TA)
	$(CC) $(SOURCES_TA) $(FLAGS) $(LIBS) -c -o $(FILE_TA)

clean :
	rm -f $(FILE_WPR) $(FILE_CMD) $(FILE_TA)

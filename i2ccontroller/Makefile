#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    I2C Communication
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS = -lpthread -lwiringPi -lrt -I$(SAILINGROBOTS_HOME)

SOURCES_I2C = I2CController.cpp
HEADERS_I2C = I2CController.h
FILE_I2C = I2CController.o

HEADERS = $(HEADERS_I2C)
SOURCES = $(SOURCES_I2C)

all : $(FILE_I2C)

$(FILE_I2C) : $(SOURCES_I2C) $(HEADERS_I2C)
	$(CC) $(SOURCES_I2C) $(FLAGS) $(LIBS) -c -o $(FILE_I2C)

clean :
	rm -f $(FILE_I2C)

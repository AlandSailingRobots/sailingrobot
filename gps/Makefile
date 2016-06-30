#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    gps
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS = -lgps -I$(SAILINGROBOTS_HOME)

SOURCES_GPS = GPSReader.cpp
HEADERS_GPS = GPSReader.h GPS.h
FILE_GPS = GPSReader.o

SOURCES_MOCKGPS = MockGPSReader.cpp
HEADERS_MOCKGPS = MockGPSReader.h
FILE_MOCKGPS = MockGPSReader.o

HEADERS = $(HEADERS_GPS) $(HEADERS_MOCKGPS)
SOURCES = $(SOURCES_GPS) $(SOURCES_MOCKGPS)

all : $(FILE_GPS) $(FILE_MOCKGPS) $(FILE_GPS_THREAD)

$(FILE_GPS) : $(SOURCES_GPS) $(HEADERS_GPS)
	$(CC) $(SOURCES_GPS) $(FLAGS) $(LIBS) -c -o $(FILE_GPS)

$(FILE_MOCKGPS) : $(SOURCES_MOCKGPS) $(HEADERS_MOCKGPS)
	$(CC) $(SOURCES_MOCKGPS) $(FLAGS) $(LIBS) -c -o $(FILE_MOCKGPS)


clean :
	rm -f $(FILE_GPS) $(FILE_MOCKGPS) $(FILE_GPS_THREAD)

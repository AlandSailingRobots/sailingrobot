CC = g++
CFLAGS = -g -pedantic -std=c++14 -I $(SAILINGROBOTS_HOME)
FILES_PARSER = src/xmlparser.cpp pugi/*.cpp
EXECUTABLE = parser

SOURCES_XML_LOG = src/xml_log.cpp
HEADERS_XML_LOG = src/xml_log.h
FILE_XML_LOG = XML_log.o

SOURCES_PUGI_XML = pugi/*.cpp
HEADERS_PUGI_XML = pugi/*.hpp
FILE_PUGI_XML = pugi.o

all : $(FILE_PUGI_XML) $(FILE_XML_LOG)

$(FILE_PUGI_XML) : $(SOURCES_PUGI_XML) $(HEADERS_PUGI_XML)
	$(CC) $(SOURCES_PUGI_XML) $(CFLAGS) -c -o $(FILE_PUGI_XML)

$(FILE_XML_LOG) : $(SOURCES_XML_LOG) $(HEADERS_XML_LOG)
	$(CC) $(SOURCES_XML_LOG) $(CFLAGS) -c -o $(FILE_XML_LOG)

parser : $(FILES_PARSER)
	$(CC) $(CFLAGS) $(FILES_PARSER) -o $(EXECUTABLE) 

clean :
	rm -f $(FILE_PUGI_XML) $(FILE_XML_LOG)
	rm -f parser
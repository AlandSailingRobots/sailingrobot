#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    coursecalculation
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -Wall -pedantic -Werror -std=c++14
LIBS = -I $(SAILINGROBOTS_HOME)

SOURCES_CC = CourseCalculation.cpp
HEADERS_CC = CourseCalculation.h
FILE_CC = CourseCalculation.o

SOURCES_CM = CourseMath.cpp
HEADERS_CM = CourseMath.h
FILE_CM = CourseMath.o


all : $(FILE_CC) $(FILE_CM)

$(FILE_CC) : $(SOURCES_CC) $(HEADERS_CC)
	$(CC) $(SOURCES_CC) $(FLAGS) $(LIBS) -c -o $(FILE_CC)

$(FILE_CM) : $(SOURCES_CM) $(HEADERS_CM)
	$(CC) $(SOURCES_CM) $(FLAGS) $(LIBS) -c -o $(FILE_CM)

clean :
	rm -f $(FILE_CC) $(FILE_CM)

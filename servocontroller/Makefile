#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    servocontroller
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS = -g -pedantic -std=c++14
LIBS =

SOURCES_MAESTRO = MaestroController.cpp
HEADERS_MAESTRO = MaestroController.h Actuator.h
FILE_MAESTRO = MaestroController.o

SOURCES_MAESTRO_MOCK = MockMaestroController.cpp
HEADERS_MAESTRO_MOCK = MockMaestroController.h
FILE_MAESTRO_MOCK = MockMaestroController.o

SOURCES_SERVO = ServoObject.cpp
HEADERS_SERVO = ServoObject.h
FILE_SERVO = ServoObject.o

SOURCES_SERVO_MOCK = MockServoObject.cpp
HEADERS_SERVO_MOCK = MockServoObject.h
FILE_SERVO_MOCK = MockServoObject.o

SOURCES_SENSOR = SensorObject.cpp
HEADERS_SENSOR = SensorObject.h
FILE_SENSOR = SensorObject.o

HEADERS = $(HEADERS_MAESTRO) $(HEADERS_SERVO) $(HEADERS_SENSOR) $(HEADERS_MAESTRO_MOCK) $(HEADERS_SERVO_MOCK)
SOURCES = $(SOURCES_MAESTRO) $(SOURCES_SERVO) $(SOURCES_SENSOR) $(SOURCES_MAESTRO_MOCK) $(SOURCES_SERVO_MOCK)

all : $(FILE_MAESTRO) $(FILE_SERVO) $(FILE_SENSOR) $(FILE_MAESTRO_MOCK) $(FILE_SERVO_MOCK)

$(FILE_MAESTRO) : $(SOURCES_MAESTRO) $(HEADERS_MAESTRO)
	$(CC) $(SOURCES_MAESTRO) $(FLAGS) $(LIBS) -c -o $(FILE_MAESTRO)

$(FILE_MAESTRO_MOCK) : $(SOURCES_MAESTRO_MOCK) $(HEADERS_MAESTRO_MOCK)
	$(CC) $(SOURCES_MAESTRO_MOCK) $(FLAGS) $(LIBS) -c -o $(FILE_MAESTRO_MOCK)
	
$(FILE_SERVO) : $(SOURCES_SERVO) $(HEADERS_SERVO)
	$(CC) $(SOURCES_SERVO) $(FLAGS) $(LIBS) -c -o $(FILE_SERVO)

$(FILE_SERVO_MOCK) : $(SOURCES_SERVO_MOCK) $(HEADERS_SERVO_MOCK)
	$(CC) $(SOURCES_SERVO_MOCK) $(FLAGS) $(LIBS) -c -o $(FILE_SERVO_MOCK)
	
$(FILE_SENSOR) : $(SOURCES_SENSOR) $(HEADERS_SENSOR)
	$(CC) $(SOURCES_SENSOR) $(FLAGS) $(LIBS) -c -o $(FILE_SENSOR)


clean :
	rm -f $(FILE_MAESTRO) $(FILE_SERVO) $(FILE_SENSOR) $(FILE_MOCK)

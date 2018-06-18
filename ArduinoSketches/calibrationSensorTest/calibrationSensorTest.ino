/*
 * File: calibrationSensorTest.ino
 * 
 * Command used for calibration: PHMID, PHLOW, PHHIGH
 *                               CONDDRY, CONDLOW, CONDHIGH
 *                               
 * For reading the values of the PH sensor, you have to quickly use the command READPH two times,
 * otherwise it prints only 0.0.
 * 
 * Reading of the conductivity sensor is working.
 * 
 * Encountered a problem for compiling when the package CanBus are present in the Arduino libraries.
 * One way to solve it is to suppress the CAN folder containing th same header file than needed here,
 * and there might also be another include not working from one of the libraries. Just commenting it
 * did the job last time.
 */

#include <Wire.h>
#include <SoftwareSerial.h>

#include <Canbus.h>
#include <MsgParsing.h>
#include <CanMessageHandler.h>


#define I2C_ADDRESS_PH 99
#define I2C_ADDRESS_CONDUCTIVETY 100

#define NO_ERRORS 0

#define ERROR_SENSOR_PH_SYNTAX 1                    // Syntax error. Happens if we send wrong command to sensor
#define ERROR_SENSOR_PH_NOT_READY 2                 // Not ready. Means we did not wait long enough for sensor to actually get a reading
#define ERROR_SENSOR_PH_NO_DATA 3                   // No data from sensor. Probably sensor failure? Check specifications from sensor documentation

#define ERROR_SENSOR_CONDUCTIVETY_SYNTAX 4          // Syntax error. Happens if we send wrong command to sensor
#define ERROR_SENSOR_CONDUCTIVETY_NOT_READY 5       // Not ready. Means we did not wait long enough for sensor to actually get a reading
#define ERROR_SENSOR_CONDUCTIVETY_NO_DATA 6         // No data from sensor. Probably sensor failure? Check specifications from sensor documentation

#define ERROR_SENSOR_TEMPERATURE_SYNTAX 7           // Syntax error. Happens if we send wrong command to sensor
#define ERROR_SENSOR_TEMPERATURE_NOT_READY 8        // Not ready. Means we did not wait long enough for sensor to actually get a reading
#define ERROR_SENSOR_TEMPERATURE_NO_DATA 9 // No data from sensor. Probably sensor failure? Check specifications from sensor documentation

#define PH 0
#define CON 1

enum {
    SENSOR_PH,
    SENSOR_CONDUCTIVETY,
};

const char* SENSOR_COMMAND_SLEEP = "Sleep";

const char* SENSOR_COMMAND_READ = "R";

const char* PH_CAL_MID = "Cal,mid,7";
const char* PH_CAL_LOW = "Cal,low,4";
const char* PH_CAL_HIGH = "Cal,high,10";

const char* COND_CAL_DRY = "Cal,dry";
const char* COND_CAL_LOW = "Cal,low,12880";
const char* COND_CAL_HIGH = "Cal,high,80000";



const int I2C_ADRESSES[] = {I2C_ADDRESS_PH,
                            I2C_ADDRESS_CONDUCTIVETY};

const int RESPONSE_STATUS_SUCCESS = 1;
const int RESPONSE_STATUS_SYNTAX_ERROR = 2;
const int RESPONSE_STATUS_NOT_READY = 254;
const int RESPONSE_STATUS_NO_DATA = 255;

const int SENSOR_READ_TIME[] = {
        600,    // Time for PH sensor to read
        150,     // Time for Conductivety sensor to read
};

const int SENSOR_INPUT_SIZE = 20;

const int PH_PROBABLE_INTERVAL_MIN = 0;
const int PH_PROBABLE_INTERVAL_MAX = 14;

const int CONDUCTIVETY_PROBABLE_INTERVAL_MIN = 1000;
const int CONDUCTIVETY_PROBABLE_INTERVAL_MAX = 42000;

const int SENSOR_READING_TRIES = 5;


unsigned long lastReadingTimeInSeconds = 0;

long int sensorReadingIntervalInSeconds = 15;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int switch_sensor_reading=0;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    Serial.println("SETUP COMPLETE");
    inputString.reserve(50);
}

void loop()
{
       sensorCalibration();
       if (stringComplete) {
           Serial.print("Command sent: ");
           Serial.println(inputString);
           if (inputString=="PHLOW\n") {
               sendCommandToSensor(SENSOR_PH,PH_CAL_LOW);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="PHMID\n") {
               sendCommandToSensor(SENSOR_PH,PH_CAL_MID);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="PHHIGH\n") {
               sendCommandToSensor(SENSOR_PH,PH_CAL_HIGH);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="CONDLOW\n") {
               sendCommandToSensor(SENSOR_CONDUCTIVETY,COND_CAL_LOW);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="CONDDRY\n") {
               sendCommandToSensor(SENSOR_CONDUCTIVETY,COND_CAL_DRY);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="CONDHIGH\n") {
               sendCommandToSensor(SENSOR_CONDUCTIVETY,COND_CAL_HIGH);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="SWITCH_SENSOR_READING\n") {
               switch_sensor_reading = (switch_sensor_reading+1)%2; //switch beteen 0 (PH) and 1 (CON) 
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="") {
               delay(10); //have to catch this or otherwise the serial is flooded by wrong command
               stringComplete = false;
           }
           else if (inputString=="READPH\n") {
               uint8_t phResponseCodeTmp;
               float x= readSensor(SENSOR_PH,phResponseCodeTmp);
               Serial.print("Read command: ");
               Serial.println(x);
               inputString = "";
               stringComplete = false;
           }
           else if (inputString=="READCOND\n") {
               uint8_t condResponseCodeTmp;
               float x= readSensor(SENSOR_CONDUCTIVETY,condResponseCodeTmp);
               Serial.print("Read command: ");
               Serial.println(x);
               inputString = "";
               stringComplete = false;
           }
           else {
               Serial.println("Wrong command");
               inputString = "";
               stringComplete = false;
           }
       }
       delay(10);
       
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void sensorCalibration() {
    unsigned long timeNowInSeconds = millis()/1000;

    if(sensorReadingIntervalInSeconds != -1 && lastReadingTimeInSeconds + sensorReadingIntervalInSeconds < timeNowInSeconds) {
        lastReadingTimeInSeconds = timeNowInSeconds;
          printMarineSensorData();
    }
  
}

void printMarineSensorData (){

  
    uint8_t phResponseCode, conductivetyResponseCode;

    if (switch_sensor_reading==PH){
    Serial.println("Reading PH Value");
   Serial.println(getPHValue(phResponseCode));
    }
    else {
    Serial.println("Reading Conductivity Value");
   Serial.println(getConductivety(conductivetyResponseCode));
    }
   
}



float getPHValue(uint8_t& responseStatusCode) {

    float value = readSensorWithProbableInterval(SENSOR_PH, responseStatusCode,
                                                 PH_PROBABLE_INTERVAL_MIN, PH_PROBABLE_INTERVAL_MAX);

    sendCommandToSensor(SENSOR_PH,SENSOR_COMMAND_SLEEP);
    
    return value;
}

float getConductivety(uint8_t& responseStatusCode) {

    float value = readSensorWithProbableInterval(SENSOR_CONDUCTIVETY, responseStatusCode,
                                                 CONDUCTIVETY_PROBABLE_INTERVAL_MIN, CONDUCTIVETY_PROBABLE_INTERVAL_MAX);
    sendCommandToSensor(SENSOR_CONDUCTIVETY,SENSOR_COMMAND_SLEEP);

    return value;
}



void sendCommandToSensor(int I2CAdressEnum, const char* command) {
    delay(50);
    Wire.beginTransmission(I2C_ADRESSES[I2CAdressEnum]);
    Wire.write(command);
    Wire.endTransmission();
}

float readSensor(int I2CAdressEnum, uint8_t& responseStatusCode) {
    sendCommandToSensor(I2CAdressEnum,SENSOR_COMMAND_READ);

    delay(SENSOR_READ_TIME[I2CAdressEnum]);

    Wire.requestFrom(I2C_ADRESSES[I2CAdressEnum], SENSOR_INPUT_SIZE, 1);
    responseStatusCode = Wire.read();

    if(responseStatusCode != 1) {
        return 0;
    }

    char sensor_input[SENSOR_INPUT_SIZE]={};

    for (int i=0;Wire.available();i++) {
        sensor_input[i] = Wire.read();
        if (sensor_input[i] == 0) {
            Wire.endTransmission();
            break;
        }
    }

    return atof(sensor_input);
}

float readSensorWithProbableInterval(int I2CAdressEnum, uint8_t& responseStatusCode, int probableIntervalMin, int probableIntervalMax) {
    float value = readSensor(I2CAdressEnum, responseStatusCode);
    int i=0;
    while( (value > probableIntervalMax || value < probableIntervalMin) && i < SENSOR_READING_TRIES) {
        value = readSensor(I2CAdressEnum, responseStatusCode);
        i++;
    }
    return value;
}

int getErrorCode(uint8_t phError, uint8_t conductivetyError, uint8_t temperatureError) {
    switch (phError) {
        case RESPONSE_STATUS_NO_DATA:
            return ERROR_SENSOR_PH_NO_DATA;
        case RESPONSE_STATUS_NOT_READY:
            return ERROR_SENSOR_PH_NOT_READY;
        case RESPONSE_STATUS_SYNTAX_ERROR:
            return ERROR_SENSOR_PH_SYNTAX;
    }
    switch (conductivetyError) {
        case RESPONSE_STATUS_NO_DATA:
            return ERROR_SENSOR_CONDUCTIVETY_NO_DATA;
        case RESPONSE_STATUS_NOT_READY:
            return ERROR_SENSOR_CONDUCTIVETY_NOT_READY;
        case RESPONSE_STATUS_SYNTAX_ERROR:
            return ERROR_SENSOR_CONDUCTIVETY_SYNTAX;
    }
    return NO_ERRORS;
}


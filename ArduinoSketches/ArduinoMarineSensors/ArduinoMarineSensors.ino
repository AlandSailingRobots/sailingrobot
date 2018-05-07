/*Purpose: Main Arduino file for marine sensor data
 *         Gets requests and reads sensors and sends data on canbus
 *         Uses CAN-bus to receive and send data.
 *
 *
 */

#include <Wire.h>
#include <SoftwareSerial.h>

#include <Canbus.h>
#include <MsgParsing.h>
#include <CanMessageHandler.h>

#define I2C_ADDRESS_PH 99
#define I2C_ADDRESS_CONDUCTIVETY 100
#define I2C_ADDRESS_TEMPERATURE 102

#define CHIP_SELECT_PIN 10

enum {
    SENSOR_PH,
    SENSOR_CONDUCTIVETY,
    SENSOR_TEMPERATURE
};

const char* SENSOR_COMMAND_SLEEP = "Sleep";

const char* SENSOR_COMMAND_READ = "R";

const int I2C_ADRESSES[] = {I2C_ADDRESS_PH,
                            I2C_ADDRESS_CONDUCTIVETY,
                            I2C_ADDRESS_TEMPERATURE};

const int RESPONSE_STATUS_NOT_CONNECTED = 0;
const int RESPONSE_STATUS_SUCCESS = 1;
const int RESPONSE_STATUS_SYNTAX_ERROR = 2;
const int RESPONSE_STATUS_NOT_READY = 254;
const int RESPONSE_STATUS_NO_DATA = 255;

const int SENSOR_READ_TIME[] = {
        900,    // Time for PH sensor to read
        600,     // Time for Conductivety sensor to read
        600    // Time for Temperature sensor to read
};

const int SENSOR_INPUT_SIZE = 20;

const int PH_PROBABLE_INTERVAL_MIN = 5;
const int PH_PROBABLE_INTERVAL_MAX = 8;

const int CONDUCTIVETY_PROBABLE_INTERVAL_MIN = 2000;
const int CONDUCTIVETY_PROBABLE_INTERVAL_MAX = 17000;

const int TEMPERATURE_PROBABLE_INTERVAL_MIN = -2;
const int TEMPERATURE_PROBABLE_INTERVAL_MAX = 35;

const int SENSOR_READING_TRIES = 5;

CanbusClass Canbus;

unsigned long lastReadingTimeInSeconds = 0;

long int sensorReadingIntervalInSeconds = 15;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    if(Canbus.Init(CHIP_SELECT_PIN)) {
        Serial.println("CAN bus initialized.");
    }

    Serial.println("SETUP COMPLETE");
}

void loop()
{
    checkCanbusFor (400);
    handleSensorReadingTimer();
}

void handleSensorReadingTimer() {
    unsigned long timeNowInSeconds = millis()/1000;

    if(sensorReadingIntervalInSeconds != -1 && lastReadingTimeInSeconds + sensorReadingIntervalInSeconds < timeNowInSeconds) {
        lastReadingTimeInSeconds = timeNowInSeconds;
        sendMarineSensorData();
    }
}

void sendMarineSensorData (){

    CanMessageHandler messageHandler(MSG_ID_MARINE_SENSOR_DATA);

    uint8_t phResponseCode, conductivetyResponseCode, temperatureResponseCode;

    messageHandler.encodeMappedMessage(SENSOR_PH_DATASIZE, getPHValue(phResponseCode),
                                       SENSOR_PH_INTERVAL_MIN, SENSOR_PH_INTERVAL_MAX);

    messageHandler.encodeMappedMessage(SENSOR_CONDUCTIVETY_DATASIZE, getConductivety(conductivetyResponseCode),
                                       SENSOR_CONDUCTIVETY_INTERVAL_MIN, SENSOR_CONDUCTIVETY_INTERVAL_MAX);

    messageHandler.encodeMappedMessage(SENSOR_TEMPERATURE_DATASIZE, getTemperature(temperatureResponseCode) ,
                                       SENSOR_TEMPERATURE_INTERVAL_MIN, SENSOR_TEMPERATURE_INTERVAL_MAX);
    messageHandler.setErrorMessage(getErrorCode(phResponseCode, conductivetyResponseCode, temperatureResponseCode));

    CanMsg marineSensorData = messageHandler.getMessage();

    Canbus.SendMessage(&marineSensorData);
}

void checkCanbusFor (int timeMs){
    int startTime= millis();
    int timer = 0;
    while (timer < timeMs){
        if (Canbus.CheckForMessages()) {
            CanMsg msg;
            Canbus.GetMessage(&msg);
            processCANMessage (msg);
        }
        timer = millis() - startTime;
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

float getTemperature(uint8_t& responseStatusCode) {

    float value = readSensorWithProbableInterval(SENSOR_TEMPERATURE, responseStatusCode,
                                                 TEMPERATURE_PROBABLE_INTERVAL_MIN, TEMPERATURE_PROBABLE_INTERVAL_MAX);
                                     
    sendCommandToSensor(SENSOR_TEMPERATURE,SENSOR_COMMAND_SLEEP);

    return value;
}


void processCANMessage (CanMsg& msg){

    CanMessageHandler messageHandler(msg);

    if (messageHandler.getMessageId() == MSG_ID_MARINE_SENSOR_REQUEST) {
        sendMarineSensorData();
        lastReadingTimeInSeconds = millis()/1000;

        bool takeContinousReadings;
        messageHandler.getData(&takeContinousReadings, REQUEST_CONTINOUS_READINGS_DATASIZE);

        if(takeContinousReadings) {
            messageHandler.getData(&sensorReadingIntervalInSeconds, REQUEST_READING_TIME_DATASIZE);
        }
        else {
            sensorReadingIntervalInSeconds = -1;
        }
    }
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
        case RESPONSE_STATUS_NOT_CONNECTED:
            return ERROR_SENSOR_PH_NO_CONNECTION;
        case RESPONSE_STATUS_NO_DATA:
            return ERROR_SENSOR_PH_NO_DATA;
        case RESPONSE_STATUS_NOT_READY:
            return ERROR_SENSOR_PH_NOT_READY;
        case RESPONSE_STATUS_SYNTAX_ERROR:
            return ERROR_SENSOR_PH_SYNTAX;
    }
    switch (conductivetyError) {
        case RESPONSE_STATUS_NOT_CONNECTED:
            return ERROR_SENSOR_CONDUCTIVETY_NO_CONNECTION;
        case RESPONSE_STATUS_NO_DATA:
            return ERROR_SENSOR_CONDUCTIVETY_NO_DATA;
        case RESPONSE_STATUS_NOT_READY:
            return ERROR_SENSOR_CONDUCTIVETY_NOT_READY;
        case RESPONSE_STATUS_SYNTAX_ERROR:
            return ERROR_SENSOR_CONDUCTIVETY_SYNTAX;
    }
    switch (temperatureError) {
        case RESPONSE_STATUS_NOT_CONNECTED:
            return ERROR_SENSOR_TEMPERATURE_NO_CONNECTION;
        case RESPONSE_STATUS_NO_DATA:
            return ERROR_SENSOR_TEMPERATURE_NO_DATA;
        case RESPONSE_STATUS_NOT_READY:
            return ERROR_SENSOR_TEMPERATURE_NOT_READY;
        case RESPONSE_STATUS_SYNTAX_ERROR:
            return ERROR_SENSOR_TEMPERATURE_SYNTAX;
    }
    return NO_ERRORS;
}

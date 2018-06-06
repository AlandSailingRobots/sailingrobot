/*Purpose: Main Arduino file for current sensor data
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

// Fill in with current sensors addresses
#define I2C_ADDRESS_CUR_SENSOR_1 9997
#define I2C_ADDRESS_CUR_SENSOR_2 9998
#define I2C_ADDRESS_CUR_SENSOR_3 9999

#define CHIP_SELECT_PIN 10

enum {
    CUR_SENSOR_1,
    CUR_SENSOR_2,
    CUR_SENSOR_3
};


const int I2C_ADRESSES[] = {I2C_ADDRESS_CUR_SENSOR_1,
                            I2C_ADDRESS_CUR_SENSOR_2,
                            I2C_ADDRESS_CUR_SENSOR_3};

// Do we want response status?
const int RESPONSE_STATUS_NOT_CONNECTED = 0;

// Might be useful later
const int SENSOR_READ_TIME[] = {
        900,    // Time for PH sensor to read
        600,     // Time for Conductivety sensor to read
        600    // Time for Temperature sensor to read
};

const int SENSOR_INPUT_SIZE = 20;

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

void sendCurrentSensorData (){

    CanMessageHandler messageHandler(MSG_ID_CURRENT_SENSOR_DATA);

    uint8_t curResponseCode;
Serial.println("Check encode status");
    // Create new encodeMessage func? -> trying encodeCSMessage
    Serial.println(messageHandler.encodeCSMessage(SENSOR_PH_DATASIZE, getCurrentValue(curResponseCode)));

    messageHandler.setErrorMessage(getErrorCode(curResponseCode));
    CanMsg currentSensorData = messageHandler.getMessage();
    Canbus.SendMessage(&currentSensorData);
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

float getCurrentValue(uint8_t& responseStatusCode) {

// Put the analog read and everything here
    float value = readCurrentSensor(CUR_SENSOR, responseStatusCode);

    sendCommandToSensor(SENSOR_PH,SENSOR_COMMAND_SLEEP);
    
    return value;
}

void processCANMessage (CanMsg& msg){

    CanMessageHandler messageHandler(msg);

    if (messageHandler.getMessageId() == MSG_ID_CURRENT_SENSOR_REQUEST) { // Do we want current sensor request
        currentSensorData();
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

float readSensor(int I2CAdressEnum, uint8_t& responseStatusCode) {
    sendCommandToSensor(I2CAdressEnum,SENSOR_COMMAND_READ);

    delay(SENSOR_READ_TIME[I2CAdressEnum]);

    Wire.requestFrom(I2C_ADRESSES[I2CAdressEnum], SENSOR_INPUT_SIZE, 1);
    responseStatusCode = Wire.read();

    if(responseStatusCode != 1) {
        return -1; //0
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

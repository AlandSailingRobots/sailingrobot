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
#define PIN_CUR_SENSOR_1 A1
#define PIN_VOL_SENSOR_1 A2
#define PIN_CUR_SENSOR_2 A3
#define PIN_VOL_SENSOR_2 A4

#define CHIP_SELECT_PIN 10

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

long int sensorReadingIntervalInSeconds = 1;

Float16Compressor fltCompressor;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    if(Canbus.Init(CHIP_SELECT_PIN)) {
        Serial.println("CAN bus initialized.");
    }

    Serial.println("SETUP COMPLETE");
    // Half precision float converter, IEEE754 standard
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
        sendCurrentSensorData();
    }
}

void sendCurrentSensorData (){

    CanMessageHandler messageHandler(MSG_ID_CURRENT_SENSOR_DATA);

    // Create new encodeMessage func? -> trying encodeCSMessage
    Serial.println(messageHandler.encodeMessage(CURRENT_SENSOR_CURRENT_DATASIZE, getCurrentValue()));
    Serial.println(messageHandler.encodeMessage(CURRENT_SENSOR_VOLTAGE_DATASIZE, getVoltageValue()));
    //Serial.println(messageHandler.encodeMessage(CURRENT_SENSOR_ELEMENT_DATASIZE, getElementValue()));

    //messageHandler.setErrorMessage(getErrorCode(curResponseCode));
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
        }
        timer = millis() - startTime;
    }
}

uint16_t getCurrentValue() {

// Put the analog read and everything here
    float value = 1.24353535368;
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

uint16_t getVoltageValue() {

// Put the analog read and everything here
    float value = 15.1515151515;
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

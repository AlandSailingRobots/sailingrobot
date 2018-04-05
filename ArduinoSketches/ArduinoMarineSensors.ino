/*Purpose: Main Arduino file for marine sensor data
 *         Gets requests and reads sensors and sends data on canbus
 *         Uses CAN-bus to receive and send data.
 *
 *
 */


#include <SoftwareSerial.h>
#include <Canbus.h>
#include <MsgParsing.h>

const int INT16_SIZE = 65535;
const long int INT32_SIZE = 4294967295;

const int SENSOR_TEMPERATURE_INTERVAL_MIN = -5;
const int SENSOR_TEMPERATURE_INTERVAL_MAX = 40;

const int SENSOR_PH_INTERVAL_MIN = 0;
const int SENSOR_PH_INTERVAL_MAX = 14;

const int SENSOR_CONDUCTIVETY_INTERVAL_MIN = 5;
const long int SENSOR_CONDUCTIVETY_INTERVAL_MAX = 200000;

CanbusClass Canbus;

unsigned long lastReadingTimeInSeconds = 0;

long int sensorReadingIntervalInSeconds = -1;

void setup()
{
    Serial.begin(9600);

    if(Canbus.Init(0)) {
        Serial.println("CAN bus initialized.");
    }

    Serial.println("SETUP COMPLETE");
}

void loop()
{
    checkCanbusFor (400);
    handleSensorReadingTimer();
}

float mapInterval(float val, float fromMin, float fromMax, float toMin, float toMax) {
    return (val - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
}

void handleSensorReadingTimer() {
    unsigned long timeNowInSeconds = millis()/1000;

    if(sensorReadingIntervalInSeconds != -1 && lastReadingTimeInSeconds + sensorReadingIntervalInSeconds < timeNowInSeconds) {
        lastReadingTimeInSeconds = timeNowInSeconds;
        sendMarineSensorData();
    }
}

void sendMarineSensorData (){
    CanMsg marineSensorData;
    marineSensorData.id = 711;
    marineSensorData.header.ide = 0;
    marineSensorData.header.length = 7;

    uint16_t phValue = mapInterval(getPHValue(), SENSOR_PH_INTERVAL_MIN, SENSOR_PH_INTERVAL_MAX, 0, INT16_SIZE);
    uint32_t conductivety = mapInterval(getConductivety(), SENSOR_CONDUCTIVETY_INTERVAL_MIN, SENSOR_CONDUCTIVETY_INTERVAL_MAX, 0, INT32_SIZE);
    uint16_t temperature = mapInterval(getTemperature(), SENSOR_TEMPERATURE_INTERVAL_MIN, SENSOR_TEMPERATURE_INTERVAL_MAX, 0, INT16_SIZE);

    marineSensorData.data[0] = (phValue & 0xff);
    marineSensorData.data[1] = (phValue >> 8);

    marineSensorData.data[2] = (conductivety & 0xff);
    marineSensorData.data[3] = (conductivety >> 8);
    marineSensorData.data[4] = (conductivety >> 16);
    marineSensorData.data[5] = (conductivety >> 24);

    marineSensorData.data[6] = (temperature & 0xff);
    marineSensorData.data[7] = (temperature >> 8);

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

float getPHValue() {
    // Mocked implementation
    // Range between 0 - 14
    return 5.2;
}

float getConductivety() {
    // Mocked implementation
    // Range between 5 - 200 000
    return 100000.2;
}

float getTemperature() {
    // Mocked implementation
    // Range between -5 - 40
    return 10.5;
}


void processCANMessage (CanMsg& msg){

    if (msg.id == 710) {
        sendMarineSensorData();
        lastReadingTimeInSeconds = millis()/1000;

        if(msg.data[0]) {
            sensorReadingIntervalInSeconds = ((long int)msg.data[4]<<24 | (long int)msg.data[3]<<16 | msg.data[2]<<8 | msg.data[1]);
        }
        else {
            sensorReadingIntervalInSeconds = -1;
        }
    }
}

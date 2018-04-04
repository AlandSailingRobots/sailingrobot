/*Purpose: Main Arduino file for the actuator unit. 
 *         Used to control the actuators and send feedback to the navigation unit. 
 *         Uses CAN-bus to receive and send data. 
 * 
 * Developer Notes: Current sensor reading needs to be added (vlon, 20.7.17)
 *
 */


#include <SoftwareSerial.h>
#include <PololuMaestro.h>
#include <Canbus.h>
#include <MsgParsing.h>

//Values are taken so the maestro output match the behavior of the radio controller in the motor controllers 
const int RUDDER_MAESTRO_MAX_TARGET = 1900;
const int RUDDER_MAESTRO_MIN_TARGET = 1150;
const int WINGSAIL_MAESTRO_MAX_TARGET = 1950;
const int WINGSAIL_MAESTRO_MIN_TARGET = 1110;

// A signal of e.g. 1200 matches the target 4800 in the
// Maestro Configurations
const int MAESTRO_SIGNAL_MULTIPLIER = 4;

// Rudder should go from -30 to +30 degrees
// which gives an effective range of 60.
const int MAX_RUDDER_ANGLE = 30;

//Windsail should go from -13 to 13 degrees
//range is 26
const int MAX_WINGSAIL_ANGLE = 13;

const double INT16_SIZE = 65535;

const int RUDDER_MIN_FEEDBACK = 278;
const int RUDDER_MAX_FEEDBACK = 358;
const int WINGSAIL_MIN_FEEDBACK =  360;
const int WINGSAIL_MAX_FEEDBACK = 980;

const int RUDDER_MAESTRO_CHANNEL = 0;
const int WINGSAIL_MAESTRO_CHANNEL = 2;

const int RUDDER_FEEDBACK_PIN = A6;
const int WINGSAIL_FEEDBACK_PIN = A4;
const int RADIO_CONTROLL_OFF_PIN = A8;

const int SENSOR_TEMPERATURE_INTERVAL_MIN = -5;
const int SENSOR_TEMPERATURE_INTERVAL_MAX = 40;

const int SENSOR_PH_INTERVAL_MIN = 0;
const int SENSOR_PH_INTERVAL_MAX = 14;

/* On boards with a hardware serial port available for use, use
that port to communicate with the Maestro. For other boards,
create a SoftwareSerial object using pin 10 to receive (RX) and
pin 11 to transmit (TX). */
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else

  SoftwareSerial maestroSerial(19, 18); //10, 11 for UNO ||19, 18 for MEGA
#endif

/* Next, create a Maestro object using the serial port.
Uncomment one of MicroMaestro or MiniMaestro below depending
on which one you have. */
MicroMaestro maestro(maestroSerial);
//MiniMaestro maestro(maestroSerial);

CanbusClass Canbus;

unsigned long lastReadingTimeInSeconds = 0;

int sensorReadingIntervalInSeconds = -1;

void setup()
{
  
  pinMode(RUDDER_FEEDBACK_PIN, INPUT);
  pinMode(WINGSAIL_FEEDBACK_PIN, INPUT);
  pinMode (RADIO_CONTROLL_OFF_PIN, INPUT);
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);

  if(Canbus.Init(0)) {
    Serial.println("CAN bus initialized.");
  }
 
  Serial.println("SETUP COMPLETE");  
}

void loop()
{
  sendArduinoData ();
  checkCanbusFor (50);
  sendFeedback ();
  checkCanbusFor (400); 
 
}

float mapInterval(float val, float fromMin, float fromMax, float toMin, float toMax) {
  return (val - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
}

void sendFeedback (){
 CanMsg feedbackMsg;
 uint16_t rudderAngle16;
 uint16_t wingsailAngle16;
 
      
      rudderAngle16 = getRudderFeedback ();
      wingsailAngle16 = getWingsailFeedback();

      feedbackMsg.id = 701;
      feedbackMsg.header.ide = 0;
      feedbackMsg.header.length = 7;  
      
      feedbackMsg.data[0] = (rudderAngle16 & 0xff);
      feedbackMsg.data[1] = (rudderAngle16 >> 8);
      feedbackMsg.data[2] = (wingsailAngle16 & 0xff);
      feedbackMsg.data[3] = (wingsailAngle16 >> 8);
      feedbackMsg.data[4] = 0;
      feedbackMsg.data[5] = 0;
      feedbackMsg.data[6] = 0;
      
      Canbus.SendMessage(&feedbackMsg);
      
}

void sendArduinoData (){
  CanMsg arduinoData;
    arduinoData.id = 702;
    arduinoData.header.ide = 0;
    arduinoData.header.length = 7;
    uint16_t RCon16 = (uint16_t) isRadioControllerUsed ();
    arduinoData.data[0] = (RCon16 & 0xff);
    arduinoData.data[1] = (RCon16 >> 8);
    arduinoData.data[2] = 0;
    arduinoData.data[3] = 0;
    arduinoData.data[4] = 0;
    arduinoData.data[5] = 0;
    arduinoData.data[6] = 0;
    
    Canbus.SendMessage(&arduinoData);
}

void handleSensorReadingTimer() {
    unsigned long timeNowInSeconds = millis()/1000;

    if(sensorReadingIntervalInSeconds != -1 && lastReadingTimeInSeconds + sensorReadingIntervalInSeconds < timeNowInSeconds) {
        lastReadingTimeInSeconds = timeNowInSeconds;
        sendArduinoData();
    }
}

void sendMarineSensorData (){
  CanMsg marineSensorData;
  marineSensorData.id = 711;
  marineSensorData.header.ide = 0;
  marineSensorData.header.length = 7;

  uint16_t phValue = mapInterval(getPHValue(), SENSOR_PH_INTERVAL_MIN, SENSOR_PH_INTERVAL_MAX, 0, INT16_SIZE);
  uint32_t conductivety = getConductivety();
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

uint16_t getRudderFeedback() {
  int feedback = analogRead(RUDDER_FEEDBACK_PIN);

//Variabales c, b1 and b2 come from formula to map to a squareroot function. Reference in Hardware documatation
  float c = -361.0000;
  float b1 =  1.8823;
  float b2 = -1.8473;
  float angle;
  if (feedback < -c){
    
    angle = b1* sqrt (-(feedback+c));
  } else {
    angle = b2* sqrt (feedback+c);
  }

  uint16_t canbusAngle = mapInterval (angle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 0, INT16_SIZE);
  return canbusAngle;
  
}

float getWingsailFeedback() {
  int feedback = analogRead(WINGSAIL_FEEDBACK_PIN);

  //Variabales c, b1 and b2 come from formula to map to a squareroot function. Reference in Hardware documatation
  float c = -450;
  float b1 = 0.7098;
  float b2 = -0.6455;
  float angle;
  if (feedback < -c){
    
    angle = b1* sqrt (-(feedback+c));
  } else {
    angle = b2* sqrt (feedback+c);
  }
  uint16_t canbusAngle = mapInterval (angle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE, 0, INT16_SIZE);
  return canbusAngle; 
}

int isRadioControllerUsed (){
  
  if (analogRead (RADIO_CONTROLL_OFF_PIN) > 250) { //Value comes from the multiplexer indicator led
    return 0;
  }
  else {
    return  INT16_SIZE/2;
  }
}

float getPHValue() {
  // Mocked implementation
  // Range between 0 - 14
  return 5;
}

float getConductivety() {
  // Mocked implementation
  // Range between 5 - 200 000
  return 100000;
}

float getTemperature() {
  // Mocked implementation
  // Range between -5 - 40
  return 10.5;
}


void processCANMessage (CanMsg& msg){
    
  if(msg.id == 700) {
    uint16_t rawCanData = (msg.data[1]<<8 | msg.data[0]);
    double rudderAngel = mapInterval (rawCanData, 0, INT16_SIZE, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
    //Serial.print("Received rudder angle: "); Serial.println(rudderAngel);
    rawCanData = (msg.data[3]<<8 | msg.data[2]);
    double wingsailAngle = mapInterval (rawCanData, 0, INT16_SIZE, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);
    //Serial.print("Received wingsail angle: "); Serial.println(wingsailAngle);

    moveRudder(rudderAngel);

    moveWingsail(wingsailAngle);
  }
  else if(msg.id == 710) {
    sendMarineSensorData();
    lastReadingTimeInSeconds = millis()/1000;

    if(msg.data[0]) {
      sensorReadingIntervalInSeconds = (msg.data[3]<<24 | msg.data[2]<<16 | msg.data[1]<<8 | msg.data[0]);
    }
    else {
      sensorReadingIntervalInSeconds = -1;
    }
  }
}

void moveRudder(double angleToSet) {
  float target = mapInterval(angleToSet, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 
                      RUDDER_MAESTRO_MIN_TARGET, RUDDER_MAESTRO_MAX_TARGET);
  
  maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*MAESTRO_SIGNAL_MULTIPLIER);
  maestro.getErrors(); //Used to clear any errors on the maestro inorder for it not to lock up
}

void moveWingsail(double angleToSet) {
  float target = mapInterval(angleToSet, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE,
                  WINGSAIL_MAESTRO_MIN_TARGET, WINGSAIL_MAESTRO_MAX_TARGET);
  maestro.setTarget(WINGSAIL_MAESTRO_CHANNEL, target*MAESTRO_SIGNAL_MULTIPLIER);  
  maestro.getErrors(); //Used to clear any errors on the maestro inorder for it not to lock up
}



 



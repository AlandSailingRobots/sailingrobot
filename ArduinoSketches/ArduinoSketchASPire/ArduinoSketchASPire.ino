#include <SoftwareSerial.h>
#include <PololuMaestro.h>
#include <Canbus.h>
#include <MsgParsing.h>

const int MAESTRO_MAX_TARGET = 1900;
const int MAESTRO_MIN_TARGET = 500;

// A signal of e.g. 1200 matches the target 4800 in the
// Maestro Configurations
const int MAESTRO_SIGNAL_MULTIPLIER = 4;

// Rudder should go from -30 to +30 degrees
// which gives an effective range of 60.
const int MAX_RUDDER_ANGLE = 60;

const int RUDDER_MIN_FEEDBACK = 178;
const int RUDDER_MAX_FEEDBACK = 680;

const int RUDDER_MAESTRO_CHANNEL = 1;
const int WINDSAIL_MAESTRO_CHANNEL = 2;

const int RUDDER_FEEDBACK_PIN = A4;
const int WINGSAIL_FEEDBACK_PIN = A5;
const int RC_CONTROLL_OFF_PIN = A8;

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

double angleratio = 65535 / MAX_RUDDER_ANGLE;
double maestroCommandRatio = (MAESTRO_MAX_TARGET - MAESTRO_MIN_TARGET) / MAX_RUDDER_ANGLE;

void setup()
{
  
  pinMode(RUDDER_FEEDBACK_PIN, INPUT);
  pinMode(WINGSAIL_FEEDBACK_PIN, INPUT);
  pinMode (RC_CONTROLL_OFF_PIN, INPUT);
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


  while(Canbus.CheckForMessages()) {
    
    CanMsg msg;
    Canbus.GetMessage(&msg);
    sendArduinoData ();
    delay(250);
    sendFeedback ();
    delay(250);
    if(msg.id == 700) {
      moveRudder(msg);
      moveWingsail(msg);
      
      
     
      
      maestro.getErrors();
      
    }
    
  }
}

void moveRudder(CanMsg& msg) {
  uint16_t rawAngle = (msg.data[1]<<8 | msg.data[0]);
  double actualAngle = rawAngle / angleratio;
  Serial.print("Received rudder angle: "); Serial.println(actualAngle);
  float target = mapInterval(actualAngle, 0, 60, 
                      MAESTRO_MIN_TARGET, MAESTRO_MAX_TARGET);

  maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*4);
}

void moveWingsail(CanMsg& msg) {
  uint16_t rawAngle = (msg.data[3]<<8 | msg.data[2]);
  double actualAngle = rawAngle / angleratio;
  Serial.print("Received windsail angle: "); Serial.println(actualAngle);

  float target = mapInterval(actualAngle, 0, 60,
                  MAESTRO_MIN_TARGET, MAESTRO_MAX_TARGET);

  maestro.setTarget(WINDSAIL_MAESTRO_CHANNEL, target*4);
  
}

float getRudderFeedback() {
  return mapInterval(analogRead(RUDDER_FEEDBACK_PIN), RUDDER_MIN_FEEDBACK, RUDDER_MAX_FEEDBACK, 0, 60);
}

float getWingsailFeedback() {
  // Obviously, these values should not actually be the same as for the rudder.
  return mapInterval(analogRead(WINGSAIL_FEEDBACK_PIN), RUDDER_MIN_FEEDBACK, RUDDER_MAX_FEEDBACK, 0, 60);  
}

float mapInterval(float val, float fromMin, float fromMax, float toMin, float toMax) {
  return (val - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
}

int isRCOn (){
  
  if (analogRead (RC_CONTROLL_OFF_PIN) > 250) {
    //Serial.println("RC OFF");
    return 0;
  }
  else {
    //Serial.println("RC ON");
    return 30;
  }
}
    
void sendFeedback (){
 CanMsg feedbackMsg;
      feedbackMsg.id = 701;
      feedbackMsg.header.ide = 0;
      feedbackMsg.header.length = 7;
      float rudderAngle = getRudderFeedback();
      uint16_t angle16 = (uint16_t) mapInterval(rudderAngle, 0, 60, 0, 65535);
      feedbackMsg.data[0] = (angle16 & 0xff);
      feedbackMsg.data[1] = (angle16 >> 8);

      float wingsailAngle = getWingsailFeedback();
      angle16 = (uint16_t) mapInterval(wingsailAngle, 0, 60, 0, 65535);
      feedbackMsg.data[2] = (angle16 & 0xff);
      feedbackMsg.data[3] = (angle16 >> 8);

      feedbackMsg.data[4] = 0;
      feedbackMsg.data[5] = 0;
      feedbackMsg.data[6] = 0;
      
      Canbus.SendMessage(&feedbackMsg);
      Serial.println("Sent feedback");
}

void sendArduinoData (){
  CanMsg arduinoData;
    arduinoData.id = 702;
    arduinoData.header.ide = 0;
    arduinoData.header.length = 7;
    int RCon = isRCOn ();
    uint16_t RCon16 = (uint16_t) mapInterval(RCon, 0, 60, 0, 65535);
    arduinoData.data[0] = (RCon16 & 0xff);
    arduinoData.data[1] = (RCon16 >> 8);
    arduinoData.data[2] = 0;
    arduinoData.data[3] = 0;
    arduinoData.data[4] = 0;
    arduinoData.data[5] = 0;
    arduinoData.data[6] = 0;

    Canbus.SendMessage(&arduinoData);


    

}


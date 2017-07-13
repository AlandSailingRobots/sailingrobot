#include <SoftwareSerial.h>
#include <PololuMaestro.h>
#include <Canbus.h>
#include <MsgParsing.h>

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
const int RC_CONTROLL_OFF_PIN = A8;

const int CANBUS_FREQUENCY = 10; //Sending freqvency (Hz)

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

double rudderAngleRatio = 65535 / MAX_RUDDER_ANGLE;
double wingsailAngleRatio = 65535 / MAX_WINGSAIL_ANGLE;
//double maestroCommandRatio = (MAESTRO_MAX_TARGET - MAESTRO_MIN_TARGET) / MAX_RUDDER_ANGLE;
double delayTime = 1000/(CANBUS_FREQUENCY*2);


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
  //Serial.print ("Delay between sends ");
  //Serial.println(delayTime);
}

void loop()
{
  sendArduinoData ();
  delay (50);
  //checkCanbusFor (100);
  
  sendFeedback ();
  checkCanbusFor (400);
  
 
}

void moveRudder(CanMsg& msg) {
  uint16_t rawAngle = (msg.data[1]<<8 | msg.data[0]);
  double actualAngle = mapInterval (rawAngle, 0, INT16_SIZE, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
  Serial.print("Received rudder angle: "); Serial.println(actualAngle);
  float target = mapInterval(actualAngle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 
                      RUDDER_MAESTRO_MIN_TARGET, RUDDER_MAESTRO_MAX_TARGET);
  
  maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*4);
  Serial.println (target);
}

void moveWingsail(CanMsg& msg) {
  uint16_t rawAngle = (msg.data[3]<<8 | msg.data[2]);
  Serial.print("Received raw angle: "); Serial.println(rawAngle);
  double actualAngle = mapInterval (rawAngle, 0, INT16_SIZE, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);
  Serial.print("Received wingsail angle: "); Serial.println(actualAngle);

  float target = mapInterval(actualAngle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE,
                  WINGSAIL_MAESTRO_MIN_TARGET, WINGSAIL_MAESTRO_MAX_TARGET);

  maestro.setTarget(WINGSAIL_MAESTRO_CHANNEL, target*4);
  //Serial.println (target*4);
}

uint16_t getRudderFeedback() {
  //Serial.println (analogRead(RUDDER_FEEDBACK_PIN));
  int feedback = analogRead(RUDDER_FEEDBACK_PIN);
   //-361.0000    1.8823   -1.8473
  float c = -361.0000;
  float b1 =  1.8823;
  float b2 = -1.8473;
  float angle;
  //Serial.println (feedback);
  if (feedback < -c){
    
    angle = b1* sqrt (-(feedback+c));
  } else {
    angle = b2* sqrt (feedback+c);
  }
  Serial.println (angle);
  uint16_t dataAngle = mapInterval (angle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE, 0, INT16_SIZE);
  //Serial.println (INT16_SIZE);
  return dataAngle;
  
}

float getWingsailFeedback() {
  // Obviously, these values should not actually be the same as for the rudder.
  int feedback = analogRead(WINGSAIL_FEEDBACK_PIN);
  float c = -450;
  float b1 = 0.7098;
  float b2 = -0.6455;
  float angle;
  //Serial.println (feedback);
  if (feedback < -c){
    
    angle = b1* sqrt (-(feedback+c));
  } else {
    angle = b2* sqrt (feedback+c);
  }
  uint16_t dataAngle = mapInterval (angle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE, 0, INT16_SIZE);
  //Serial.println (angle);
  return dataAngle;
  
  
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
      uint16_t angle16 = getRudderFeedback ();
      feedbackMsg.data[0] = (angle16 & 0xff);
      feedbackMsg.data[1] = (angle16 >> 8);
      angle16 = getWingsailFeedback();
      feedbackMsg.data[2] = (angle16 & 0xff);
      feedbackMsg.data[3] = (angle16 >> 8);

      feedbackMsg.data[4] = 0;
      feedbackMsg.data[5] = 0;
      feedbackMsg.data[6] = 0;
     
      Canbus.SendMessage(&feedbackMsg);
      //Serial.println("Sent feedback");
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
void checkCanbusFor (int timeMs){
  int startTime= millis();
  int timer = 0;
  while (timer < timeMs){
    if (Canbus.CheckForMessages()) {
    
    CanMsg msg;
    Canbus.GetMessage(&msg);
    //Serial.println (msg.id);
    //Serial.println ("ok");
      if(msg.id == 700) {
      
        moveRudder(msg);
        moveWingsail(msg);
      
        maestro.getErrors();
      }
    }
    timer = millis() - startTime;
    //Serial.println(timer);
  }
}


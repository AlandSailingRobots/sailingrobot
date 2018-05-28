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
#include <CanUtility.h>
#include <CanMessageHandler.h>

#define CHIP_SELECT_PIN 49

//Values are taken so the maestro output match the behavior of the radio controller in the motor controllers 
const int RUDDER_MAESTRO_MAX_TARGET = 1900;
const int RUDDER_MAESTRO_MIN_TARGET = 1150;
const int WINGSAIL_MAESTRO_MAX_TARGET = 1950;
const int WINGSAIL_MAESTRO_MIN_TARGET = 1110;

// A signal of e.g. 1200 matches the target 4800 in the
// Maestro Configurations
const int MAESTRO_SIGNAL_MULTIPLIER = 4;

const int RUDDER_MIN_FEEDBACK = 278;
const int RUDDER_MAX_FEEDBACK = 358;
const int WINGSAIL_MIN_FEEDBACK =  360;
const int WINGSAIL_MAX_FEEDBACK = 980;

const int RUDDER_MAESTRO_CHANNEL = 0;
const int WINGSAIL_MAESTRO_CHANNEL = 2;

const int RUDDER_FEEDBACK_PIN = A6;
const int WINGSAIL_FEEDBACK_PIN = A4;
const int RADIO_CONTROLL_OFF_PIN = A8;


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




void setup()
{

  pinMode(RUDDER_FEEDBACK_PIN, INPUT);
  pinMode(WINGSAIL_FEEDBACK_PIN, INPUT);
  pinMode (RADIO_CONTROLL_OFF_PIN, INPUT);
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);

  if(Canbus.Init(CHIP_SELECT_PIN)) {
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

void sendFeedback (){
  CanMessageHandler messageHandler(MSG_ID_AU_FEEDBACK);

  messageHandler.encodeMappedMessage(RUDDER_ANGLE_DATASIZE, getRudderFeedback(),MIN_RUDDER_ANGLE,MAX_RUDDER_ANGLE);
  messageHandler.encodeMappedMessage(WINGSAIL_ANGLE_DATASIZE, getWingsailFeedback(), MIN_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);

  CanMsg feedbackMsg = messageHandler.getMessage();

  Canbus.SendMessage(&feedbackMsg);
}

void sendArduinoData (){
  CanMessageHandler messageHandler(MSG_ID_RC_STATUS);
  messageHandler.encodeMessage(RADIOCONTROLLER_ON_DATASIZE, isRadioControllerUsed());

  CanMsg arduinoData = messageHandler.getMessage();
  Canbus.SendMessage(&arduinoData);
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

float getRudderFeedback() {
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

  return angle;
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

  return angle;
}

bool isRadioControllerUsed (){

  if (analogRead (RADIO_CONTROLL_OFF_PIN) > 250) { //Value comes from the multiplexer indicator led
    return false;
  }
  else {
    return true;
  }
}


void processCANMessage (CanMsg& msg){
  CanMessageHandler messageHandler(msg);

  if(messageHandler.getMessageId() == MSG_ID_AU_CONTROL) {
    double rudderAngle;
    messageHandler.getMappedData(&rudderAngle, RUDDER_ANGLE_DATASIZE, MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE);

    double wingsailAngle;
    messageHandler.getMappedData(&wingsailAngle, WINGSAIL_ANGLE_DATASIZE, MIN_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE);

    moveRudder(rudderAngle);

    moveWingsail(wingsailAngle);
  }
}

void moveRudder(double angleToSet) {
  float target = CanUtility::mapInterval(angleToSet, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE,
                      RUDDER_MAESTRO_MIN_TARGET, RUDDER_MAESTRO_MAX_TARGET);
  
  maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*MAESTRO_SIGNAL_MULTIPLIER);
  maestro.getErrors(); //Used to clear any errors on the maestro inorder for it not to lock up
}

void moveWingsail(double angleToSet) {
  float target = CanUtility::mapInterval(angleToSet, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE,
                  WINGSAIL_MAESTRO_MIN_TARGET, WINGSAIL_MAESTRO_MAX_TARGET);
  maestro.setTarget(WINGSAIL_MAESTRO_CHANNEL, target*MAESTRO_SIGNAL_MULTIPLIER);  
  maestro.getErrors(); //Used to clear any errors on the maestro inorder for it not to lock up
}



 



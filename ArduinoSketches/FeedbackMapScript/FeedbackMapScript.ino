/*
 * This script is a modyfied copy of the main ASPireArduinoScript as of July 2017. 
 * It is used to get the analog feedback read for the angles to be used in MATLAB vector in GetRudder- and WingssailParamaters MATLAB files.
 * It is only to be used for testing the angles and not to be used during operation.
 */



#include <SoftwareSerial.h>
#include <PololuMaestro.h>
#include <Canbus.h>
#include <MsgParsing.h>
#include <CanUtility.h>

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
int stepLen = 4;
const int arraySize = calcArraySize (stepLen, MAX_WINGSAIL_ANGLE);

int maxAngle;
int angles [1+2*MAX_RUDDER_ANGLE];
int feedback [1+2*MAX_RUDDER_ANGLE];
int dataSize;
int delayTime = 2000;
String mode;


void setup() {

  pinMode(RUDDER_FEEDBACK_PIN, INPUT);
  pinMode(WINGSAIL_FEEDBACK_PIN, INPUT);
  pinMode (RC_CONTROLL_OFF_PIN, INPUT);
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);
  
  moveWingsail (-13);
  Serial.println ("Ready To Test");

  //testLoop (MAX_WINGSAIL_ANGLE, stepLen, angles, feedback);
  //printVectors ();
  
}

void loop() {
 
  // put your main code here, to run repeatedly:
  mode = Serial.readString(); 
  if (mode=="wingsail"){
     stepLen = 1;
     
     dataSize  = testLoop (MAX_WINGSAIL_ANGLE, stepLen, angles, feedback, "wingsail");
     printVector ("angle" , angles, dataSize);
     printVector ("feedback", feedback, dataSize);
     mode = "";
  } else if (mode=="rudder"){
     stepLen = 5;
     dataSize  = testLoop (MAX_RUDDER_ANGLE, stepLen, angles, feedback, "rudder");
     printVector ("rudderAngle" , angles, dataSize);
     printVector ("rudderFeedback", feedback, dataSize);
     mode = "";
  } else if (mode=="test"){
     stepLen = 1;
     maxAngle = 5;
     dataSize  = testLoop (maxAngle, stepLen, angles, feedback, "test");
     printVector ("angle" , angles, dataSize);
     printVector ("feedback", feedback, dataSize);
     mode = "";
  }
  moveWingsail (-MAX_WINGSAIL_ANGLE);
  moveRudder (-MAX_RUDDER_ANGLE);
    
  
  
}


void moveWingsail(int angle) {
 

  float target = CanUtility::mapInterval(angle, -MAX_WINGSAIL_ANGLE, MAX_WINGSAIL_ANGLE,
                  WINGSAIL_MAESTRO_MIN_TARGET, WINGSAIL_MAESTRO_MAX_TARGET);

  maestro.setTarget(WINGSAIL_MAESTRO_CHANNEL, target*4);
 
}

void moveRudder(int angle) {

  float target = CanUtility::mapInterval(angle, -MAX_RUDDER_ANGLE, MAX_RUDDER_ANGLE,
                      RUDDER_MAESTRO_MIN_TARGET, RUDDER_MAESTRO_MAX_TARGET);
  
  maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*4);
  
}

//The test loop is used to get all relevant data, the meny isteps is inorder to get a symetric data vector around zero 
int  testLoop (int maxAngle, int stepLen, int angles[], int feedback[], String mode){
    int i = 0;
    int angle = -maxAngle;
    int nextAngle = angle + stepLen;
    logData (i,angle, angles, feedback, mode);
    i++;
  while (nextAngle < 0){
    angle += stepLen;
    logData (i,angle, angles, feedback, mode);
    i++;
    nextAngle +=stepLen;
  }
  logData (i,0, angles, feedback, mode);
  i++;
  if (nextAngle == 0){
    angle += stepLen;
    
  }
  while (nextAngle < maxAngle){
    angle += stepLen;
    logData (i,angle, angles, feedback, mode);
    i++;
    nextAngle +=stepLen;
  }
  if (angle != maxAngle){
     logData (i,maxAngle, angles, feedback, mode);
    }
 return i-1;
 }


void printVector (String vname, int vector [], int len) {

  //Serial.println(len);
  Serial.print(vname);
  Serial.print(" = [");
  for (int i = 0; i< len; i++){
    Serial.print (vector[i]);
    Serial.print(", ");
    
  }
  Serial.print (vector[len]);
  Serial.println("]';");
  
  
}

void printVectors () {


  Serial.print(" angle = [");
  for (int i = - MAX_WINGSAIL_ANGLE; i< MAX_WINGSAIL_ANGLE; i++){
    Serial.print (angles[i+MAX_WINGSAIL_ANGLE]);
    Serial.print(", ");
    
  }
  Serial.print (angles[MAX_WINGSAIL_ANGLE*2]);
  Serial.println("]';");
    Serial.print("feedback = [");
  for (int i = - MAX_WINGSAIL_ANGLE; i< MAX_WINGSAIL_ANGLE; i++){
    Serial.print (feedback[i+MAX_WINGSAIL_ANGLE]);
    Serial.print(", ");
    
  }
  Serial.print (feedback[MAX_WINGSAIL_ANGLE*2]);
  Serial.println("]';");
  
}
void logData (int i,int angle, int angles [], int feedback[], String mode){
   if (mode == "wingsail"){
  
    angles[i] = angle;
    moveWingsail (angle);
    delay (delayTime);
    feedback[i] = analogRead(WINGSAIL_FEEDBACK_PIN );
  } 
  else if (mode == "rudder"){
  
    angles[i] = angle;
    moveRudder (angle);
    delay (delayTime);
    feedback[i] = analogRead(RUDDER_FEEDBACK_PIN );
  }
  else if (mode == "test"){
     angles[i] = angle;
     Serial.println (angle);
     feedback[i] = analogRead(RUDDER_FEEDBACK_PIN );
     Serial.println(feedback[i]);
  }
}



/* Program that allows the user to set the windwane motor in a specific position 
 * Position is given in degrees with 0 degrees is having the windwane straight back.
 */

// Include necessary libraries for compilation
#include <SPI.h>
#include <mcp_can.h>
#include <AHM36A.h>
#include <math.h>

// Motor 1
const int dir1Pin = 3;
const int dir2Pin = 4;
const int speedPin = 22; // Needs to be a PWM pin to be able to control motor speed

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 53; //Arduino Mega, Uno should be 9/10
MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin

const int encoderCanNodeId = 5;
unsigned char start = 0x01;
unsigned char flagRecv = 0;

unsigned long lastValue = 0;
unsigned int windwanePosition = 0;

void setup() {
  Serial.begin(9600);

  setupCAN();  // TODO; check with HW
  
  pinMode(dir1Pin,OUTPUT);
  pinMode(dir2Pin,OUTPUT);
  pinMode(speedPin,OUTPUT);

  Encoder.read_pos(encoderCanNodeId);
}

void loop() {
  Serial.println("What to do? c - print current pos, m - move to pos, r - read pos again");
  wait();
  char command = Serial.read();

  if (command == 'c') {
    Serial.print("Current position: ");
    Serial.println( (windwanePosition<105) ? 360+(windwanePosition-105) : windwanePosition-105 );
    
  } else if(command == 'm') {
    int targetPos = getTarget();
   // Serial.print("Target: "); Serial.println("Current position: ");
    rotateWindvane(targetPos);
    
  } else if(command == 'r') {
    Encoder.read_pos(encoderCanNodeId);
  }
}

void rotateWindvane(int targetPos) {     

  int anglesToTurn = offsetAngle(targetPos) - windwanePosition;
  int dir = 0;
  
  if (anglesToTurn < 0) {
    anglesToTurn = -anglesToTurn;
    dir = 2;
  }
  else if (anglesToTurn > 180) {
    anglesToTurn = 360 - anglesToTurn;
    dir = (dir == 2) ? 0 : 2; // "invert" dir, 0->2, 2->0
  }

  int timeToTurn = (anglesToTurn/17.5)*1000;
  motorAction(dir, timeToTurn);
  Encoder.read_pos(encoderCanNodeId);
  delay(50); // delay to let the encoder send the position

  // check if the target was missed, in that case adjust it
  int angleDifference = abs(offsetAngle(targetPos) - windwanePosition);
  if (angleDifference > 180) {
    angleDifference = 360 - angleDifference;
  }

  // assume a number of degrees is close enough
  if (abs(angleDifference) > 2) {
    rotateWindvane(targetPos);
  }
}

int offsetAngle(int angle) {
  return (angle + 105) % 360;
}

int getTarget() {
  Serial.println("Enter position (0-360) to which you want to move");
  wait();

  return Serial.parseInt();
}

void setServoPos() {
  
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long id = 0;
    unsigned char len = 0;         
    unsigned char buf[8];
    // read data,  len: data length, buf: data buf
    CAN.readMsgBufID(&id, &len, buf);
  
    if (len == 8) {
      const int UNITS_PER_REVOLUTION = 24576;
      unsigned long value;
      int diff = 0;
      memcpy(&value, &buf[4], 4);
      if (lastValue != 0){
        diff = value - lastValue;
      }
      lastValue = value;
  
      value = value % UNITS_PER_REVOLUTION;
      value = map(value, 0, UNITS_PER_REVOLUTION, 0, 360);
      windwanePosition = value;
    }
  }
}

void motorAction(int action, unsigned long time){
    switch(action){
    case 0: // forward
    analogWrite(speedPin, 255);    
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    break;
    case 1: // stop
    analogWrite(speedPin, 0);
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    break;
    case 2: // reverse    
    analogWrite(speedPin, 255);    
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    break;
    default:
    digitalWrite(speedPin, LOW);
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
  }
  delay(time);
  digitalWrite(speedPin, LOW);
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW); 
}

void setupCAN(){
  Serial.println("CAN Bus Encoder Interface");
  unsigned char status = CAN.begin(CAN_250KBPS, 1);
  while (status != CAN_OK){
      Serial.println("CAN BUS Shield init fail");
      delay(100);
      Serial.println("Init CAN BUS Shield again");
      status = CAN.begin(CAN_250KBPS, 1);
  }
  Serial.println("CAN BUS Shield init ok!");

  attachInterrupt(digitalPinToInterrupt(2), setServoPos, FALLING); // start interrupt
  
  Encoder.init(&CAN); 
  Encoder.message_NMT(encoderCanNodeId,start);

  Encoder.start_node(encoderCanNodeId);
  //Encoder.read_pos(encoderCanNodeId);
}

void wait(){
   while(!Serial.available()){
    // Loop until input received
  }
}


void debugAngleCalculation() {
  
  Serial.println("Enter new angles");
  while(!Serial.available());
  
  int a1 = Serial.parseInt();
  int a2 = Serial.parseInt();
  
  const int MAX_RUDDER_ANGLE = 30; // assume max rudder angle is 30 degrees
  int degreesOffCourse = abs(((a1 - a2) + 180) % 360 - 180);
  float command = -1;
  
  // boat is heading too far starboard
  if (degreesOffCourse > 180) {
  degreesOffCourse += 360;
  
  // limit the calculation to the max possible angle to give
  if (degreesOffCourse > MAX_RUDDER_ANGLE) {
    degreesOffCourse = MAX_RUDDER_ANGLE;
  }
  
  command = degreesOffCourse * -0.5 / MAX_RUDDER_ANGLE + 0.5;
  }
  // boat is heading too far port
  else {
  // limit the calculation to the max possible angle to give
  if (degreesOffCourse > MAX_RUDDER_ANGLE) {
    degreesOffCourse = MAX_RUDDER_ANGLE;
  }
  
  command = degreesOffCourse * 0.5 / MAX_RUDDER_ANGLE + 0.5;
  }
  
  Serial.print("Diff: "); Serial.println(degreesOffCourse);
  Serial.print("Command: "); Serial.println(command);
}

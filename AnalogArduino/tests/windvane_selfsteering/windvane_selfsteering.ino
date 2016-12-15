/* Controlling the selfsteering of the windvane */

#include <PololuMaestro.h>
#include <SPI.h>
#include <mcp_can.h>
#include <AHM36A.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(19, 18); //10, 11 for UNO ||19, 18 for MEGA
#endif

// Arduino Mega, Uno should be 9/10
const int SPI_CS_PIN = 53; 
MCP_CAN CAN(SPI_CS_PIN);

MicroMaestro maestro(maestroSerial);

// constants for cleaner code
const unsigned short CHANNEL_ACTUATOR = 3;
const unsigned short CHANNEL_CLUTCH = 1;
const unsigned short WV_PIN_1 = 3;
const unsigned short WV_PIN_2 = 4;
const unsigned short WV_SPEED_PIN = 22;
const int ENCODER_CAN_NODE_ID = 5;
const int UNITS_PER_REVOLUTION = 24576;
const int TARGET_POS_THRESHOLD = 2;
const int ENCODER_OFFSET_ANGLES = 105;
const int MAX_RUDDER_ANGLE = 30;


// global variables used
bool selfsteering_state = false;
bool last_selfsteering_state = false;
unsigned int windChangeThreshold = 0;
unsigned int windDirection = 0;
unsigned int lastWindDirection = 0;
unsigned int heading = 0;
unsigned int courseToSteer = 0;
unsigned int lastCourseToSteer = 0;
unsigned int windwanePosition = 0;


void setup() {
  Serial.begin(9600);
  maestroSerial.begin(9600);
  setupCAN();

  pinMode(WV_PIN_1,OUTPUT);
  pinMode(WV_PIN_2,OUTPUT);
  pinMode(WV_SPEED_PIN,OUTPUT);

  Serial.println("SETUP COMPLETE");
}

void loop() {
  setLoopValues(); // for testing only, remove later
  
  if (selfsteering_state && !last_selfsteering_state) {
    selfSteeringTurnedOn();
  }
  if (selfsteering_state && last_selfsteering_state) {
    selfSteeringOn();
  }
  if (!selfsteering_state && last_selfsteering_state) {
    selfSteeringTurnedOff();
  }
  if (!selfsteering_state && !last_selfsteering_state) {
    selfSteeringOff();
  }

  lastWindDirection = windDirection;
  lastCourseToSteer = courseToSteer;
  last_selfsteering_state = selfsteering_state;
  delay(500);
}

void selfSteeringTurnedOn() {
  moveActuator(0.9);
  setClutchState(false); // turn off the saildrive clutch
}

void selfSteeringOn() {
  if (abs(windDirection - lastWindDirection) > windChangeThreshold || courseToSteer != lastCourseToSteer) {
    rotateWindvane(courseToSteer - windDirection);
  }
}

void selfSteeringTurnedOff() {
  moveActuator(0.1);
  setClutchState(true); // turn on the saildrive clutch    
}

void selfSteeringOff() {
  float clutchCommand = calculateClutchCommand();
  moveSaildrive(clutchCommand);
  
  // windwane is locked here, but rotate it up with the wind to give minimum wind resistance
  rotateWindvane(windDirection); 
}

/* 
 *  Function to use while testing for setting values
 */
void setLoopValues() {
  Serial.println("Enter new wind direction, course to steer, heading, and selfsteering state (separated by comma)");
  while(!Serial.available());
  
  windDirection = Serial.parseInt();
  courseToSteer = Serial.parseInt();
  heading = Serial.parseInt();
  selfsteering_state = Serial.parseInt();
  Serial.print(windDirection); Serial.print(" "); Serial.print(courseToSteer); Serial.print(" ");
  Serial.print(heading); Serial.print(" "); Serial.println(selfsteering_state);
}

/* Placeholder function for the time being..
 * Controls the power for the saildrive clutch
 * param: false = power off, true = power on
 */
void setClutchState(bool state) {
  if (state) {
    // TODO: turn on saildrive
    
  }
  else {
    // TODO: turn off saildrive
  
  }
}

/* Help function that calculates the command
 * to give the saildrive clutch
 * returns: the command to give, between 0 and 1.
 *          0 assumed to be full port
 *          0.5 assumed to be straight ahead
 *          1 assumed to be full starboard
 */
float calculateClutchCommand() {
  int degreesOffCourse = abs(((heading - courseToSteer) + 180) % 360 - 180); // limit angle to 0-360
  float command = -1;
  
  // boat is heading too far starboard
  if (degreesOffCourse > 180) {
    degreesOffCourse = 360 - degreesOffCourse;

    // limit the calculation to the max possible angle to give
    if (degreesOffCourse > MAX_RUDDER_ANGLE) {
      degreesOffCourse = MAX_RUDDER_ANGLE;
    }

    command = degreesOffCourse * -0.5 / MAX_RUDDER_ANGLE + 0.5; // "maps" the degrees off course to a command 0-0.5
  }
  // boat is heading too far port
  else {
    // limit the calculation to the max possible angle to give
    if (degreesOffCourse > MAX_RUDDER_ANGLE) {
      degreesOffCourse = MAX_RUDDER_ANGLE;
    }

    command = degreesOffCourse * 0.5 / MAX_RUDDER_ANGLE + 0.5; // "maps" the degrees off course to a command 0.5-1
  }

  return command;
}

/* Move the actuator
 * param: the position to move to, 0 - fully withrawn, 1 - fully extended
 * 0.5 wont apparently put it in middle, only go fully extent but slower?
 */
void moveActuator(float targetPos) {
  // these numbers are the values for min and max positions of the actuator
  int realPos = (targetPos) * (4520 - 6520) + 6520;
  maestro.setTarget(CHANNEL_ACTUATOR, realPos);
}

/* Move the saildrive
 * param: the position to move to, 0 - full port, 1 - fully starboard
 */
void moveSaildrive(float targetPos) {
  // these numbers are the values for min and max positions of the saildrive
  int realPos = (targetPos) * (5824 - 7424) + 7424;
  maestro.setTarget(CHANNEL_CLUTCH, realPos);
}

/* Rotate the windvane
 * param: targetPos - target position in degrees, 0-360 
 * Windwane is pointing straight back at 0 degrees.
 */
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
  Encoder.read_pos(ENCODER_CAN_NODE_ID);
  delay(50); // delay to let the encoder send the position

  // check if the target was missed, in that case adjust it
  int angleDifference = abs(offsetAngle(targetPos) - windwanePosition);
  if (angleDifference > 180) {
    angleDifference = 360 - angleDifference;
  }

  // assume a number of degrees is close enough
  if (abs(angleDifference) > TARGET_POS_THRESHOLD) {
    rotateWindvane(targetPos);
  }
}

/* Help function for offsetting an angle in order to make
 * 0 deg pointing straight back.
 * i.e. when you set the windwane to point straight back you set it to 0 deg
 *      but if you look at the position after that it will say 105
 */
int offsetAngle(int angle) {
  return (angle + ENCODER_OFFSET_ANGLES) % 360;
}

/* Help function for moving the windvane
 * params: action - direction to rotate
 *         time - how long to rotate
 */
void motorAction(int action, unsigned long time){
    switch(action){
    case 0: // clockwise
    analogWrite(WV_SPEED_PIN, 255);    
    digitalWrite(WV_PIN_1, LOW);
    digitalWrite(WV_PIN_2, HIGH);
    break;
    case 1: // stop
    analogWrite(WV_SPEED_PIN, 0);
    digitalWrite(WV_PIN_1, LOW);
    digitalWrite(WV_PIN_2, HIGH);
    break;
    case 2: // counter-clockwise    
    analogWrite(WV_SPEED_PIN, 255);    
    digitalWrite(WV_PIN_1, HIGH);
    digitalWrite(WV_PIN_2, LOW);
    break;
    default:
    digitalWrite(WV_SPEED_PIN, LOW);
    digitalWrite(WV_PIN_1, LOW);
    digitalWrite(WV_PIN_2, LOW);
  }
  
  delay(time);
  digitalWrite(WV_SPEED_PIN, LOW);
  digitalWrite(WV_PIN_1, LOW);
  digitalWrite(WV_PIN_2, LOW); 
}

/* Setup method for can-bus */
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
  Encoder.message_NMT(ENCODER_CAN_NODE_ID, 0x01);

  Encoder.start_node(ENCODER_CAN_NODE_ID);
  Encoder.read_pos(ENCODER_CAN_NODE_ID);
}

/* Interrupt function called when you request a read from the encoder
 * Parses out the read value, modifies it to position in degrees
 */
void setServoPos() {
  
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long id = 0;
    unsigned char len = 0;         
    unsigned char buf[8];
    // read data,  len: data length, buf: data buf
    CAN.readMsgBufID(&id, &len, buf);
  
    if (len == 8) {
      unsigned long value;
      memcpy(&value, &buf[4], 4);

      // convert the read value to degrees
      value = value % UNITS_PER_REVOLUTION;
      value = map(value, 0, UNITS_PER_REVOLUTION, 0, 360);
      windwanePosition = value;
    }
  }
}

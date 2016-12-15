/* Manually controlling clutch or antuator */

#include <PololuMaestro.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(19, 18); //10, 11 for UNO ||19, 18 for MEGA
#endif


// Uncomment below depending on which one you have.
MicroMaestro maestro(maestroSerial);
//MiniMaestro maestro(maestroSerial);

unsigned short CHANNEL_ACTUATOR = 3;
unsigned short CHANNEL_CLUTCH = 1;

void setup() {
  maestroSerial.begin(9600);
  Serial.begin(9600);
  Serial.println("SETUP COMPLETE");
}

void loop() {
  Serial.print("Current position: "); Serial.println(maestro.getPosition(CHANNEL_CLUTCH));
  
  Serial.println("Clutch (c) or Antuator (a) ?");
  while(!Serial.available()){ }
  char command = Serial.read();

  if (command == 'c') {
    Serial.println("Where do you want to move clutch (0-1)?");
    while(!Serial.available()){ }
    float target = Serial.parseFloat();
    if (target >= 0 && target <= 1) {
      moveClutch(target);
    }
  }
  else if (command == 'a') {
    Serial.println("Where do you want to move antuator (0-1) ?");
    Serial.println("(Does not work properly, will move to either endpoint)");
    while(!Serial.available()){ }
    float target = Serial.parseFloat();
    if (target >= 0 && target <= 1) {
      moveActuator(target);
    }
  }
}

/* Move the actuator, expects a value between 0 and 1 */
void moveActuator(float targetPos) {  
  int realPos = (targetPos) * (4520 - 6624) + 6624;
  maestro.setTarget(CHANNEL_ACTUATOR, realPos);
}

/* Move the saildrive clutch, expects a value between 0 and 1 */
void moveClutch(float targetPos) {
  int realPos = (targetPos) * (5824 - 7424) + 7424;
  maestro.setTarget(CHANNEL_CLUTCH, realPos);
}

float waitForInput(){
  while(!Serial.available()){ }
  return Serial.parseFloat();
}

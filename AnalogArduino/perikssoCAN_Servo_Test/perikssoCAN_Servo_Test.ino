#include <SoftwareSerial.h>
#include <PololuMaestro.h>
#include <Canbus.h>

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
  
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);

  if(Canbus.Init(0)) {
    Serial.println("CAN bus initialized.");
  }
 
  Serial.println("SETUP COMPLETE");  
  
  // INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, const INT8U *buf, bool wait_sent=true);   /* send buf  */
}

void loop()
{


  while(Canbus.CheckForMessages()) {
    CanMsg msg;
    Canbus.GetMessage(&msg);
    if(msg.id == 700) {
      uint16_t rawAngle = (msg.data[1]<<8 | msg.data[0]);
      double actualAngle = rawAngle / angleratio;
      Serial.print("Received rudder angle: "); Serial.println(actualAngle);
      float target = (actualAngle * (MAESTRO_MAX_TARGET - MAESTRO_MIN_TARGET)/60)
                      + MAESTRO_MIN_TARGET;

      Serial.print("Servo target : "); Serial.println(target*4);      
      maestro.setTarget(RUDDER_MAESTRO_CHANNEL, target*4);
      Serial.print("Servo getPos : "); Serial.println(maestro.getPosition(RUDDER_MAESTRO_CHANNEL));

      rawAngle = (msg.data[3]<<8 | msg.data[2]);
      actualAngle = rawAngle / angleratio;
      Serial.print("Received windsail angle: "); Serial.println(actualAngle);
      target = (actualAngle * (MAESTRO_MAX_TARGET - MAESTRO_MIN_TARGET)/60)
                      + MAESTRO_MIN_TARGET;

      maestro.setTarget(WINDSAIL_MAESTRO_CHANNEL, target*4);

      delay(5000);


      double rudderFeedback = analogRead(A4);
      rudderFeedback = (rudderFeedback * (MAESTRO_MAX_TARGET - MAESTRO_MIN_TARGET)/4095)
                          + MAESTRO_MIN_TARGET;

      rudderFeedback = (rudderFeedback - MAESTRO_MIN_TARGET)
                        * (60)/(MAESTRO_MAX_TARGET-MAESTRO_MIN_TARGET);              
      Serial.print("Calculated rudder angle from feedback: "); Serial.println(rudderFeedback);
      Serial.print("Rudder feedback: "); Serial.println(analogRead(A4));
      //Serial.print("Wingsail feedback: "); Serial.println(analogRead(A5));
      maestro.getErrors();
    }
  }
}

// Set a target as percentage to a given channel on the Maestro
void moveServo(int channel, float target) {
  float input =  target * 40.96;
  maestro.setTarget(channel, input);
}


#include <mcp_can.h>
#include <CANWinch.h>
#include <Servo.h> 

Servo SpeedController; // for the RC motor speed controller

#define servoPin 6               // signal for ESC servo cable
// Do not use pins 4 and 5, 
// GND and 5V BEC from the ESC are connected here,
// for more mechanical attachment.
#define feedbackPin A5           // HallSensor 0-5V output
#define currentSensorPin A4      // ACS709
#define motorTemperaturePin A3   // LM35DT
#define reducerTemperaturePin A2 // LM35DT

// protection of thermal fuses in circuit
const float   fuseCurrent    = 30.0; // fuse setting. This code limits speed to respect this fuse value.
const float   maxTemp        = 55.0; // maximum temperature allowed inside winch case
const float   slowDownTemp   = 45.0; // temperature at which the motor current is reduced to limit more heating

const byte    speedDeadBand  =    6; // dead band of the command for motor speed controller

const int     mainLoopPeriod =    10; // in milli seconds

// software end-stops for rope position sensor
// Keep a minimum of 100 ticks as safe distance from 0 and 1024 end values.
// This is to take into account that the Hall sensor does not go all the way to zero, 
// and the play in the little worm-reducer connecting the sensor.
const int feedbackMin = 100;
const int feedbackMax = 923;


// variable for continous calibration of the Hall current sensor
float currentSensorOffset = 0.0;

// motor state
enum motorState_enum 
  {AWAIT_NEUTRAL_COMMAND,           // At startup, first speed command must be neutral. This is a savety feature.
   TURNING_MINUS, HALTING_MINUS,    // Do not allow reverse while still not stopped, and respect a delay.
   HALTED, 
   HALTING_PLUS, TURNING_PLUS} 
            motorState = AWAIT_NEUTRAL_COMMAND;
signed char       lastSpeed = 0;    // previous speed setting for comparison
unsigned long motorStopTime = 0;    // time since motor gets a HALT command

int  loopCount;    // counts main loop() executions for actions not executed during every loop()

byte debugData[8]; // variable to store debugging data to be transmitted by CAN bus using sendDebugData()
signed char got, gave, give;




float getMotorCurrent(){
// return current going through the Hall effect current sensor

   return ((analogRead(currentSensorPin)/1024.0)*5.0-2.5)/0.028 // 28mV/A; 5V range; Zero current at 2.5 V; 10 bits resolution (1024 ticks)
          - currentSensorOffset;
}

float getMotorTemp(){
  
  analogRead(motorTemperaturePin);   // Relatively low output impedance sensor:
                                     // this is used to switch the analog multiplexer of the ADC to the correct pin.
                                     // Additional delay may be necessary.
  return (analogRead(motorTemperaturePin)/1024.0)*5.0*100.0;
}

float getReducerTemp(){

  analogRead(reducerTemperaturePin); // Relatively low output impedance sensor:
                                     // this is used to switch the analog multiplexer of the ADC to the correct pin.
                                     // Additional delay may be necessary.
  return (analogRead(reducerTemperaturePin)/1024.0)*5.0*100.0;
  
}

unsigned getPositionFeedback(){
 
 return analogRead(feedbackPin);
 
}


void calibrateCurrentSensor(){
// Call this regularily, when motor is stopped for a while, to calibrate the Hall current sensor.
// Must be called about 50 times before you can start using the sensor.

   float current;

   current = getMotorCurrent();
 
   currentSensorOffset += current/10.0; // accumulate offset: pwm is zero/off, value should be zero
  
}
 
float lastError = 1.0;

// Adapt speed as necessary to respect given current limit.
signed char reducedSpeedForCurrentLimiting(signed char speed){

  float current; // measured current  
  float   ratio; // ratio of current over measured fuseCurrent

  current = getMotorCurrent();
  current = abs(current);
  
  ratio   = current/fuseCurrent;
  
  float error = (ratio - 1.0);
  // When hand tuning the PD coeffients, desactivate other limiting functions, most imortantly the accelaration limitation.
  //speedReductionFactor -=  error * 0.007 + (error - lastError) * 0.006;
  //speedReductionFactor -=  error * 0.006 + (error - lastError) * 0.006;
  //speedReductionFactor -=  error * 0.005 + (error - lastError) * 0.020;
  //speedReductionFactor -=  error * 0.001 + (error - lastError) * 0.0200;
  //speedReductionFactor -=  error * 0.002 + (error - lastError) * 0.0200;
  //speedReductionFactor -=  error * 0.002 + (error - lastError) * 0.0300;
  //speedReductionFactor -=  error * 0.002 + (error - lastError) * 0.0500;
  //speedReductionFactor -=  error * 0.002 + (error - lastError) * 0.050;
  //speedReductionFactor -=  error * 0.002 + (error - lastError) * 0.100;
  // Mainloop = 1; speedReductionFactor -=  error * 0.002 + (error - lastError) * 0.040;
  // loop =10 speedReductionFactor -=  error * 0.05 + (error - lastError) * 0.1;
  //speedReductionFactor -=  error * 0.004 + (error - lastError) * 0.020; 8-11
// sous charge 20 kg 10A  speedReductionFactor -=  error * 0.05 + (error - lastError) * 0.10;
//  speedReductionFactor -=  error * 0.004 + (error - lastError) * 0.1;
//  speedReductionFactor -=  error * 0.04 + (error - lastError) * 0.2;

  if (error > 0.5) { // Too much current. The fuse could blow.
    SpeedController.write(90); // stop the motor   
    Winch.setMainSailWinchState((byte) current, 
                              (byte) getMotorTemp(),
                              (byte) getReducerTemp(),
                              (byte) (getPositionFeedback()/4));  
    // stop everything for 2 seoconds to let things cool down.    
    delay(2000);
    return 0;
  }
  
  // no problem, keep speed
  return speed; 
}

// Adapt speed as necessary to keep temperature at acceptable level
signed char reducedSpeedForTemperatureLimiting(signed char speed){
  
  float temperature;  
  
//  float temperature, t1, t2;
//  debugData[0]= (byte) round(abs(getMotorCurrent()));  
//  debugData[1]= (byte) (t1=getMotorTemp());
//  debugData[2]= (byte) (t2=getReducerTemp());
//  debugData[3]= (byte) (getPositionFeedback()/4);
//temperature = max(t1, t2);

  temperature = max(getMotorTemp(), getReducerTemp());
  if (temperature >= maxTemp) 
     return 0; // stop motor
  else if (temperature > slowDownTemp) // slow down progressively with temperature
     return (byte) round(((float) speed) * (1 - (0.8 * (temperature - slowDownTemp) / (maxTemp - slowDownTemp)))); // minimum speed = 20 %
  else return speed;   
  
}

// Adapt speed as necessary to respect end stops.
signed char reducedSpeedForEndStops(signed char speed){

  int          feedback; // position feedback

  feedback = getPositionFeedback();
  if ((feedback > feedbackMin) && (feedback < feedbackMax))
    // still between endstops:
    { 
    // slow down approaching end stops (25 is a minimum speed):
    speed = constrain(speed, -20 - 0.8 * abs(feedback-feedbackMin), 20 + 0.8 * abs(feedbackMax-feedback));
    }
  else if (((feedback <= feedbackMin) && (speed > 0)) || 
           ((feedback >= feedbackMax) && (speed < 0)))  
    ; // heading to the good direction, returning between endstops, keep speed
  else 
    speed = 0; // ...over the feedbacklimit, don't move in this direction.
    
  return speed;
}

void setSpeed(signed char speed){
// Set motor speed from -128 to +127
// Positive speeds leed to growing positionFeedback.
  
  // unblock motor after startup at first neutral command
  if ((motorState == AWAIT_NEUTRAL_COMMAND) && (abs(speed) < speedDeadBand)) motorState = HALTED;  

  
  speed = reducedSpeedForEndStops(speed);
  speed = reducedSpeedForCurrentLimiting(speed);
//debugData[0] = (speed >= 0 ? speed : 0);
//debugData[1] = (speed < 0 ? -speed : 0);
  speed = reducedSpeedForTemperatureLimiting(speed);

  // Do not allow reverse, by disallowing reverse while running or still in HALTING state:
  if ((((motorState==TURNING_PLUS)  || (motorState==HALTING_PLUS))  and (speed < 0)) ||
      (((motorState==TURNING_MINUS) || (motorState==HALTING_MINUS)) and (speed > 0)))
    speed = 0;
  
  // limit acceleration, but allow fast stop 
  // This is to smooth commands to avoid to big currents that would trigger current limiting for fuse protection.
  // Too slower changes make produce more heat dissipated by the ESC.
  // /3 works for the fuse protection
  if ((speed > 0) && (speed > lastSpeed + mainLoopPeriod/2)) speed = lastSpeed + mainLoopPeriod/2;
  if ((speed < 0) && (speed < lastSpeed - mainLoopPeriod/2)) speed = lastSpeed - mainLoopPeriod/2;

  if (motorState == AWAIT_NEUTRAL_COMMAND) speed = 0;
  
  //debugData[0]= (byte) round(abs(getMotorCurrent()));  
  //debugData[1]= (byte) (getMotorTemp());
  //debugData[2]= (byte) (getReducerTemp());
  //debugData[3]=speed;
  // map speed and direction to servo angles  
  SpeedController.write(map(speed, -128, 127, 0, 180));


  if (motorState == HALTED) calibrateCurrentSensor();

  // update motor state:
  if ((motorState == HALTED) and (abs(speed) > speedDeadBand)){
    // change state depending on direction 
    if (speed > 0) motorState = TURNING_PLUS;
    else           motorState = TURNING_MINUS;
  }
    
  if ((motorState == TURNING_PLUS) && (abs(speed) <= speedDeadBand)){
    motorState = HALTING_PLUS;
    motorStopTime = millis();
  }
  if ((motorState == TURNING_MINUS) && (abs(speed) <= speedDeadBand)){
    motorState = HALTING_MINUS;
    motorStopTime = millis();
  }
    
  if ((motorState == HALTING_PLUS) && (abs(speed) > speedDeadBand))
    motorState = TURNING_PLUS;
  if ((motorState == HALTING_MINUS) && (abs(speed) > speedDeadBand))
    motorState = TURNING_MINUS;

  if (((motorState == HALTING_PLUS) || (motorState == HALTING_MINUS))
      && ((millis() - motorStopTime) > 500))
    motorState = HALTED;
    
  if ((motorState == HALTED) && (speed > speedDeadBand))
    motorState = TURNING_PLUS;  
  if ((motorState == HALTED) && (speed < -speedDeadBand))
    motorState = TURNING_MINUS;  
 
  // keep speed for comparison with next speed command
  lastSpeed     = speed;
  }   

void setup()
{
  Serial.begin(9600);
  // wait for serial connection to host beeing established.
  // *** Only for debugging,
  // desactivate this for normal operation without host computer! */
  // while (0 && !Serial) {
  //  ; // Wait for serial port to connect. This is needed for Leonardo/micro/CANinterfacers's "Serial" only, not for "Serial".
  // }
  Serial.println("Here is the mainsail node.");

  SpeedController.attach(servoPin);
  SpeedController.write(90);
 
  // Calibrate zero current, now that we now the current is zero.
  int i=50;
  while (i-->=0) {
     calibrateCurrentSensor();
     delay(1);
  }

//XXX Watchdog reset and stop motor, if speed command isn't transmitted anymore, or if the code execution get stuck.

    give = -50;
}

void loop()
{
  // rope speed and direction
  signed char speed;
  
  delay(mainLoopPeriod);
  loopCount++;
 
  speed = Winch.getMainSailWinchSpeed();
 
  // Set speed to zero if unallowed values are tranmitted.
  if ((speed < -128) || (speed > 127))
     speed = 0;
 
  //setSpeed(10);

  // From time to time send state information.
  if (0 ||(loopCount % 100) == 0) {

    Serial.print("Got: ");
    got=Winch.getMainSailWinchSpeed();
    gave=got+10;
    Serial.println(got,HEX);
    Serial.print("Gave:");
    Serial.println(give,HEX);
    Winch.setMainSailWinchSpeed(give);
    Serial.print("Got2: ");
    got=Winch.getMainSailWinchSpeed();
    Serial.println(got,HEX);
    Winch.setMainSailWinchSpeed(got);


    
    Serial.print("Before:");
    Serial.print(Winch.getMainSailWinchCurrent(), HEX);                              
    Serial.print(" | ");
    Serial.print(Winch.getMainSailWinchMotorTemp(), HEX);                          
    Serial.print(" | ");
    Serial.print(Winch.getMainSailWinchReducerTemp(), HEX);                          
    Serial.print(" | ");
    Serial.println(Winch.getMainSailWinchPosition(), HEX);  
    
    Winch.setMainSailWinchState((byte) round(abs(getMotorCurrent())), 
                              (byte) getMotorTemp(),
                              (byte) getReducerTemp(),
                              (byte) 0x70);  
    Serial.print("After: ");
    Serial.print(Winch.getMainSailWinchCurrent(), HEX);                              
    Serial.print(" | ");
    Serial.print(Winch.getMainSailWinchMotorTemp(), HEX);                          
    Serial.print(" | ");
    Serial.print(Winch.getMainSailWinchReducerTemp(), HEX);                          
    Serial.print(" | ");
    Serial.println(Winch.getMainSailWinchPosition(), HEX);                              
    
  }      
  
  // XXX watchdog reset if speed timestamp is new enough
  
}


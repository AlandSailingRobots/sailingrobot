

#include "CANWinch.h"

const byte  _rudderAngleId       = 49;
const byte _mainSailWinchSpeedId = 50;
const byte _mainSailWinchStateId = 33;
const byte _headSailWinchSpeedId = 51;
const byte _rcOverwriteId        = 60;


WinchClass::WinchClass()
{
	
}
//assign CAN object
void WinchClass::init(MCP_CAN* _CAN)
{  
	CAN = _CAN;
}

// MainSailWinchSpeed
void WinchClass::setMainSailWinchSpeed(signed char value)
{
  byte data[7];
  
  // update local copies
  _setLocalMainSailWinchSpeed(value);
  
  // update remote copies
  data[0] = (byte) value;
  _sendCAN(_mainSailWinchSpeedId, data, 1);
}

void WinchClass::_setLocalMainSailWinchSpeed(signed char value)
{
  _mainSailWinchSpeed = constrain(value, -128, 127);
  _mainSailWinchSpeedTimestamp = millis();
}

signed char WinchClass::getMainSailWinchSpeed(){
  return _mainSailWinchSpeed;
}

// MainSailWinchState
void WinchClass::setMainSailWinchState(byte current, byte motorTemp, byte reducerTemp, byte position)
{
  byte data[4];
  
  // update local copies
  _setLocalMainSailWinchState(current, motorTemp, reducerTemp, position);
  
  // update remote copies
  data[0] = (byte) current;
  data[1] = (byte) motorTemp;
  data[2] = (byte) reducerTemp;
  data[3] = (byte) position;
  _sendCAN(_mainSailWinchStateId, data, 4);
}

void WinchClass::_setLocalMainSailWinchState(byte current, byte motorTemp, byte reducerTemp, byte position)
{
  _mainSailWinchCurrent     = constrain(current, 0, 254);
  _mainSailWinchMotorTemp   = constrain(motorTemp, 0, 254);
  _mainSailWinchReducerTemp = constrain(reducerTemp, 0, 254);
  _mainSailWinchPosition    = constrain(position, 0, 254);

  _mainSailWinchStateTimestamp= millis();
}

byte WinchClass::getMainSailWinchCurrent(){
  return _mainSailWinchCurrent;
}

byte WinchClass::getMainSailWinchMotorTemp(){
  return _mainSailWinchMotorTemp;
}

byte WinchClass::getMainSailWinchReducerTemp(){
  return _mainSailWinchReducerTemp;
}

byte WinchClass::getMainSailWinchPosition(){
  return _mainSailWinchPosition;
}


void WinchClass::_sendCAN(uint16_t id, byte data[8], byte length){
// send one byte to the bus
  //Serial.println("Send CAN");
  
digitalWrite(13, 1);
  noInterrupts(); // reserve SPI for us, do not let interrupt use the SPI now
  CAN->sendMsgBuf(id, 0, 0, length, data);
  interrupts();
digitalWrite(13, 0);  

}


WinchClass Winch;
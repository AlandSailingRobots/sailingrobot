/*
  Bus.h - Library 
  Quick and dirty library to make use of the CAN bus 
  
  First created by Bernt Weber, June 1, 2014.
  
  Released under BSD license
  
*/

#ifndef Bus_h
#define Bus_h

#include "Arduino.h"
#include <mcp_can.h>

class WinchClass
{
  public:
		MCP_CAN* CAN;
		
    WinchClass();

		void init(MCP_CAN*);
    void setMainSailWinchSpeed(signed char value);
    // values from -128 to +127
    signed char getMainSailWinchSpeed();
    // values from -128 to +127

    void setMainSailWinchState(byte courrent, byte motorTemp, byte reducerTemp, byte position);
    // values from -128 to +127
    byte getMainSailWinchCurrent();
    // in degrees Celsius
    // values from 0 to 255
    byte getMainSailWinchMotorTemp();
    // in degrees Celsius
    // values from 0 to 255
    byte getMainSailWinchReducerTemp();
    // in degrees Celsius
    // values from 0 to 255
    byte getMainSailWinchPosition();
    // values from 0 to 255

    
    // The following methods should be private, but are public so that the interrupt service routine can access them.
    void _setLocalMainSailWinchSpeed(signed char value);
    void _setLocalMainSailWinchState(byte current, byte motorTemp, byte reducerTemp, byte position);
 
  private:
    volatile signed char _mainSailWinchSpeed    = 127;
    volatile unsigned long _mainSailWinchSpeedTimestamp = 0 - 1; // big value
    volatile unsigned long _mainSailWinchStateTimestamp = 0 - 1; // big value
    volatile signed char _mainSailWinchCurrent     =   0;
    volatile signed char _mainSailWinchMotorTemp   =   0;
    volatile signed char _mainSailWinchReducerTemp =   0;
    volatile signed char _mainSailWinchPosition    =   0;
    
    void _sendCAN(uint16_t id, uint8_t data[8], uint8_t length);
    
};

extern WinchClass Winch;

#endif

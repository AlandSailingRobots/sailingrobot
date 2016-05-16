/*
 * AnalogArduino.h
 *
 */

#ifndef __ANALOG_ARDUINO_H__
#define __ANALOG_ARDUINO_H__

#include <stdint.h> // uint8_t

class PressureSensorModel;

class AnalogArduino {
public:
	AnalogArduino(){};
	virtual ~AnalogArduino(){};
	// setup for the connection between Raspberry Pi and the PressureSensor
	virtual bool init()=0;
	
	virtual int getValue()=0;
	// returns m_pressure
	virtual PressureSensorModel getModel()=0;
};

#endif

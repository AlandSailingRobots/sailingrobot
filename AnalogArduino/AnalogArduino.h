/*
 * AnalogArduino.h
 *
 */

#ifndef __ANALOG_ARDUINO_H__
#define __ANALOG_ARDUINO_H__

#include <stdint.h> // uint8_t

class AnalogArduinoModel;

class AnalogArduino {
public:
	AnalogArduino(){};
	virtual ~AnalogArduino(){};
	// setup for the connection between Raspberry Pi and the Arduino
	virtual bool init()=0;

	virtual int getValue0()=0;
	// returns m_pressure
	virtual int getValue1()=0;
	// returns m_rudder
	virtual int getValue2()=0;
	// returns m_sheet
	virtual int getValue3()=0;
	// returns m_battery
	virtual void readValues()=0;
	
	virtual AnalogArduinoModel getModel()=0;
};

#endif

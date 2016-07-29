/*
 * MockPressureSensor.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef __MOCK_ANALOG_ARDUINO_H__
#define __MOCK_ANALOG_ARDUINO_H__

#include "AnalogArduino.h"

class AnalogArduinoModel;

class MockAnalogArduino:public AnalogArduino {
public:
	MockAnalogArduino();
	~MockAnalogArduino();

	// initialize module
	bool init();

	// returns pressure
	int getValue0();
	// returns rudder
	int getValue1();
	// returns sheet
	int getValue2();
	// returns battery
	int getValue3();

	void readValues();

	// returns model
	AnalogArduinoModel getModel();
};

#endif /* AnalogArduino_MOCKAnalogArduino_H_ */

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

	// returns heading
	int getValue();

	// returns model
	AnalogArduinoModel getModel();
};

#endif /* AnalogArduino_MOCKAnalogArduino_H_ */

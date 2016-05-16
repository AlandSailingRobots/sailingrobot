/*
 * MockPressureSensor.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef PRESSURESENSOR_MOCKPRESSURESENSOR_H_
#define PRESSURESENSOR_MOCKPRESSURESENSOR_H_

#include "AnalogArduino.h"

class AnalogArduinoModel;

class MockPressureSensor:public AnalogArduino {
public:
	MockPressureSensor();
	~MockPressureSensor();

	// initialize module
	bool init();

	// returns heading
	int getPressure();

	// returns model
	AnalogArduinoModel getModel();
};

#endif /* AnalogArduino_MOCKPressureSensor_H_ */

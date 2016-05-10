/*
 * MockPressureSensor.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef PRESSURESENSOR_MOCKPRESSURESENSOR_H_
#define PRESSURESENSOR_MOCKPRESSURESENSOR_H_

#include "PressureSensor.h"

class PressureSensorModel;

class MockPressureSensor:public PressureSensor {
public:
	MockPressureSensor();
	~MockPressureSensor();

	// initialize module
	bool init();

	// returns heading
	int getPressure();

	// returns model
	PressureSensorModel getModel();
};

#endif /* PressureSensor_MOCKPressureSensor_H_ */

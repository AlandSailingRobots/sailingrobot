/*
 * PressureSensor.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef PRESSURESENSOR_PRESSURESENSOR_H_
#define PRESSURESENSOR_PRESSURESENSOR_H_

#include <stdint.h> // uint8_t

class PressureSensorModel;

class PressureSensor{
public:
	PressureSensor(){};
	virtual ~PressureSensor(){};
	// setup for the connection between Raspberry Pi and the PressureSensor
	virtual bool init()=0;
	
	virtual int getPressure()=0;
	// returns m_pressure
	virtual PressureSensorModel getModel()=0;
};

#endif

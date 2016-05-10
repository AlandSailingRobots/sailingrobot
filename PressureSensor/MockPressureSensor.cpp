/*
 * MockPressureSensor.cpp
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#include "MockPressureSensor.h"
#include "models/PressureSensorModel.h"

const int PRESSURE= 513;

MockPressureSensor::MockPressureSensor() {

}

MockPressureSensor::~MockPressureSensor() {

}

bool MockPressureSensor::init(){
	return true;
}

int MockPressureSensor::getPressure(){
	return PRESSURE;
}

PressureSensorModel MockPressureSensor::getModel(){
	return PressureSensorModel(PRESSURE);
}

/*
 * MockAnalogArduino.cpp
 *
 */

#include "MockAnalogArduino.h"
#include "models/AnalogArduinoModel.h"

const int analogValue = 513;

MockAnalogArduino::MockAnalogArduino() {

}

MockAnalogArduino::~MockAnalogArduino() {

}

bool MockAnalogArduino::init(){
	return true;
}

int MockAnalogArduino::getValue(){
	return analogValue;
}

AnalogArduinoModel MockAnalogArduino::getModel(){
	return AnalogArduinoModel(analogValue);
}

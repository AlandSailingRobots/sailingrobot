/*
 * MockAnalogArduino.cpp
 *
 */

#include "MockAnalogArduino.h"
#include "models/AnalogArduinoModel.h"

const int analogValue0 = 510;
const int analogValue1 = 511;
const int analogValue2 = 512;
const int analogValue3 = 513;

MockAnalogArduino::MockAnalogArduino() {

}

MockAnalogArduino::~MockAnalogArduino() {

}

bool MockAnalogArduino::init(){
	return true;
}

int MockAnalogArduino::getValue0(){
	return analogValue0;
}

int MockAnalogArduino::getValue1(){
	return analogValue1;
}

int MockAnalogArduino::getValue2(){
	return analogValue2;
}

int MockAnalogArduino::getValue3(){
	return analogValue3;
}

AnalogArduinoModel MockAnalogArduino::getModel(){
	return AnalogArduinoModel(analogValue0, analogValue1, analogValue2, analogValue3);
}

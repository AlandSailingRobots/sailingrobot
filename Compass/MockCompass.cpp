/*
 * MockCompass.cpp
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#include "MockCompass.h"
#include "models/CompassModel.h"

const int HEADING = 100,PITCH = 200,ROLL = 300;
const int ACCELX = 5,ACCELY = 5,ACCELZ = 5;

MockCompass::MockCompass() {

}

MockCompass::~MockCompass() {

}

bool MockCompass::init(){
	return true;
}

int MockCompass::getHeading(){
	return HEADING;
}

int MockCompass::getPitch(){
	return PITCH;
}

int MockCompass::getRoll(){
	return ROLL;
}

void MockCompass::readValues(){

}

int MockCompass::getAccelX(){
	return ACCELX;
}

int MockCompass::getAccelY(){
	return ACCELY;
}

int MockCompass::getAccelZ(){
	return ACCELZ;
}

void MockCompass::readTilt(){
}

void MockCompass::readMag(){
}

void MockCompass::readAccel(){
}

void MockCompass::setOrientation(uint8_t orientation){
}

CompassModel MockCompass::getModel(){
	return CompassModel(HEADING,PITCH,ROLL,AccelerationModel(ACCELX,ACCELY,ACCELZ) );
}

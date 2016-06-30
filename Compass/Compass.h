/*
 * Compass.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef COMPASS_COMPASS_H_
#define COMPASS_COMPASS_H_

#include <stdint.h> // uint8_t

class CompassModel;

class Compass{
public:
	Compass(){};
	virtual ~Compass(){};
	// setup for the connection between Raspberry Pi and the compass
	virtual bool init()=0;
	// returns m_heading
	virtual int getHeading()=0;
	// returns m_heading
	virtual int getPitch()=0;
	// returns m_heading
	virtual int getRoll()=0;
	// returns accelX
	virtual int getAccelX()=0;
        // returns accelY
	virtual int getAccelY()=0;
        // returns accelZ
	virtual int getAccelZ()=0;
	// read heading, pitch and roll
	virtual void readValues()=0;
	// read tilt data
	virtual void readTilt()=0;
	// read magnetometer values
	virtual void readMag()=0;
	// read accelerometer values
	virtual void readAccel()=0;
	// set orientation data for correct readings
	virtual void setOrientation(uint8_t orientation)=0;
	// returns model
	virtual CompassModel getModel()=0;
};

#endif

/*
 * MockCompass.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef COMPASS_MOCKCOMPASS_H_
#define COMPASS_MOCKCOMPASS_H_

#include "Compass.h"

class CompassModel;

class MockCompass:public Compass {
public:
	MockCompass();
	~MockCompass();

	// initialize module
	bool init();

	// returns heading
	int getHeading();

	// returns pitch
	int getPitch();

	// returns roll
	int getRoll();

	// read values from module
	void readValues();

	// returns accelX
	int getAccelX();
        
        // returns accelY
	int getAccelY();
        
        // returns accelZ
	int getAccelZ();

	// read tilt data
	void readTilt();

	// read magnetometer values
	void readMag();

	// read accelerometer values
	void readAccel();

	// set orientation data for correct readings
	void setOrientation(uint8_t orientation);

	// returns model
	CompassModel getModel();
};

#endif /* COMPASS_MOCKCOMPASS_H_ */

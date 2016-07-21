/*
 * MockServoObject.h
 *
 *  Created on: Apr 24, 2015
 *      Author: sailbot
 */

#ifndef SERVOCONTROLLER_MOCKSERVOOBJECT_H_
#define SERVOCONTROLLER_MOCKSERVOOBJECT_H_

#include "ServoObject.h"

class MockServoObject:public ServoObject {
public:
	MockServoObject();
	virtual ~MockServoObject();

	void setController(Actuator * maestro);

	void setChannel(int channel);

	void setSpeed(unsigned short speed);
	void setAcceleration(unsigned short acceleration);
	void setPosition(unsigned short position);

	int getPosition();
};

#endif /* SERVOCONTROLLER_MOCKSERVOOBJECT_H_ */

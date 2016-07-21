/*
 * MockMaestroController.h
 *
 *  Created on: Apr 24, 2015
 *      Author: sailbot
 */

#ifndef SERVOCONTROLLER_MOCKMAESTROCONTROLLER_H_
#define SERVOCONTROLLER_MOCKMAESTROCONTROLLER_H_

#include "Actuator.h"

class MockMaestroController:public Actuator {
public:
	MockMaestroController();
	virtual ~MockMaestroController();
	void setPort(std::string portName);
	void writeCommand(unsigned char type, int channel, int value);
	int readRespons();
	int getError();

private:
	void openPort();
	bool isOpenPort();
};

#endif /* SERVOCONTROLLER_MOCKMAESTROCONTROLLER_H_ */

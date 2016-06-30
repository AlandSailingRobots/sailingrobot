/*
 * File:   ServoObject.h
 * Author: Joakim
 *
 * Created on 20 March 2014, 11:17
 * The main purpose for this class is to implement methods for default servo behavior.
 *
 */

#ifndef __SERVOOBJECT_H__
#define	__SERVOOBJECT_H__

#include <memory>
#include "MaestroController.h"

class ServoObject {
public:

	ServoObject();
	~ServoObject();

	// void loadConfig(cfg);
	void setController(Actuator * maestro);

	void setChannel(int channel);

	void setSpeed(unsigned short speed);
	void setAcceleration(unsigned short acceleration);
	void setPosition(unsigned short position);

	int getPosition();

private:
	int m_channel;

	int m_range;
	int m_speed;
	int m_acceleration;

	std::shared_ptr<Actuator> m_maestro;

};

#endif

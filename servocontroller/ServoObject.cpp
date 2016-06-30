#include "ServoObject.h"



ServoObject::ServoObject() {

}

ServoObject::~ServoObject() {

}

void ServoObject::setController(Actuator* maestro)
{
	m_maestro.reset(maestro); 
}

void ServoObject::setChannel(int channel)
{
	m_channel = channel;
}

void ServoObject::setSpeed(unsigned short speed) {
	m_speed = speed;
	m_maestro->writeCommand(SET_SPEED, m_channel, speed);
}

void ServoObject::setAcceleration(unsigned short acceleration) {
	m_acceleration = acceleration;
	m_maestro->writeCommand(SET_ACCELERATION, m_channel, acceleration);
}

void ServoObject::setPosition(unsigned short position) {
	m_maestro->writeCommand(SET_POSITION, m_channel, position);

}

int ServoObject::getPosition() {
	m_maestro->writeCommand(GET_POSITION, m_channel, -1);
	return m_maestro->readRespons();
}


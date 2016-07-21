#include "SensorObject.h"
#include <iostream>

SensorObject::SensorObject() {
 
}

SensorObject::~SensorObject() {

}

void SensorObject::setController(Actuator* maestro)
{
    m_maestro.reset(maestro);
}

void SensorObject::setChannel(int channel)
{
	m_channel = channel;
}

int SensorObject::getPosition()
{
	m_maestro->writeCommand(GET_POSITION, m_channel, -1);
	return m_maestro->readRespons();
}

int SensorObject::getDirection()
{
	int high = 0, low = 0;
	for (int i = 0; i < 1000; i++) {
		try {
			if (getPosition() > 100) {
				high++;
			} else {
				low++;
			}
		} catch (const char * error) {
			i--;
		}
	}
	float percent = ((float)high) / (low+high);
	return (int)(percent*360);
}

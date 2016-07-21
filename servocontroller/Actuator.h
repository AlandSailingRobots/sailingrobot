#ifndef __ACTUATOR_H__
#define	__ACTUATOR_H__

#include <string>

class Actuator {
public:
	Actuator() {};
	virtual ~Actuator() {};

	virtual void setPort(std::string portName)=0;
	virtual void writeCommand(unsigned char type, int channel, int value)=0;
	virtual int readRespons()=0;
	virtual int getError()=0;
};

#endif


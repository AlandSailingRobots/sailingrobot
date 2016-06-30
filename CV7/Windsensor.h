/*
 * Windsensor.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef CV7_WINDSENSOR_H_
#define CV7_WINDSENSOR_H_

#include <string>

class WindsensorModel;

class Windsensor{
public:
	Windsensor() {};
	virtual ~Windsensor() {};
	virtual void loadConfig(std::string portName, int baudRate)=0;
	virtual void setBufferSize(unsigned int bufferSize)=0;
	virtual void setBaudRate(unsigned int baudRate)=0;
	virtual void setPortName(std::string portName)=0;
	virtual unsigned int getBufferSize()=0;
	virtual std::string refreshData()=0;
	virtual void parseData(std::string data)=0;
	virtual float getDirection()=0;
	virtual float getSpeed()=0;
	virtual float getTemperature()=0;
	virtual bool isUseMean()=0;
	virtual void setUseMean(bool useMean)=0;
	virtual void getModel(WindsensorModel *model)=0;
};

#endif /* CV7_WINDSENSOR_H_ */

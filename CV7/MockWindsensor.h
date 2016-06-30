/*
 * MockWindsensor.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sailbot
 */

#ifndef CV7_MOCKWINDSENSOR_H_
#define CV7_MOCKWINDSENSOR_H_

#include "Windsensor.h"
#include <string>

class WindsensorModel;

class MockWindsensor:public Windsensor {
	public:
		MockWindsensor() {};
		~MockWindsensor() {};
		void loadConfig(std::string portName, int baudRate);
		void setBufferSize(unsigned int bufferSize);
		void setBaudRate(unsigned int baudRate);
		void setPortName(std::string portName);
		unsigned int getBufferSize();
		std::string refreshData();
		void parseData(std::string data);
		float getDirection();
		float getSpeed();
		float getTemperature();
		bool isUseMean();
		void setUseMean(bool useMean);
		void getModel(WindsensorModel *model);
};
#endif /* CV7_MOCKWINDSENSOR_H_ */

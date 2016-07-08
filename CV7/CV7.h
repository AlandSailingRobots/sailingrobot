#ifndef __CV7_H__
#define __CV7_H__

#include <string>
#include <vector>
#include <map>
#include "Windsensor.h"

class WindsensorModel;

class CV7:public Windsensor {
	
	public:

		CV7();
		~CV7();
		/*
			Loads the CV7 windsensor.
		*/
		bool loadConfig(std::string portName, int baudRate);
		/*
			Sets vector sizes. Must be greater than 0 and default value is 30.
		*/
		void setBufferSize(unsigned int bufferSize);
		/*
			Sets frequenzy of signal updates and restarts the readings.
		*/
		void setBaudRate(unsigned int baudRate);
		/*
			Keeps track of the current portname (should never really change after start)
		*/
		void setPortName(std::string portName);
		/*
			Returns current bufferSize value set.
		*/
		unsigned int getBufferSize();
		/*
			Gets a new reading from the sensor and adds them to the buffer vectors. Throws exception on error
		*/
		std::string refreshData();
		/*
			parses the data from windsensor and puts values in vectors
		*/
		bool parseData(std::string data);
		/*
			Returns an average wind direction value, depending on how many values that is in vector.
		*/
		float getDirection();
		/*
			Returns an average wind speed value, depending on how many values that is in vector.
		*/
		float getSpeed();
		/*
			Returns an average wind temperature value, depending on how many values that is in vector.
		*/
		float getTemperature();
		bool isUseMean();
		void setUseMean(bool useMean);
		void getModel(WindsensorModel *model);

	private:
		bool m_useMean;
		int m_fd;

		unsigned int m_bufferSize;
		unsigned int m_baudRate;
		std::string m_portName;
		
		std::vector<float> m_windDirection;
		std::vector<float> m_windSpeed;
		std::vector<float> m_windTemperature;
};

#endif

#include "CV7.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <wiringSerial.h>
#include <iostream>
#include <utility/Utility.h>
#include "UtilityLibrary.h"
#include "models/WindsensorModel.h"

CV7::CV7() {
		m_bufferSize = 30;
		m_useMean = true;
		m_portName = "";
		m_baudRate = 0;
}

CV7::~CV7() 
{
	if(m_fd < 0) {
		serialClose(m_fd);
	}
}

void CV7::loadConfig(std::string portName, int baudRate)
{	
	setPortName(portName);
	
	m_fd = serialOpen(portName.c_str(), baudRate);

	if(m_fd < 0) {
		throw "CV7::openPort: Unable to connect";
	}
}

void CV7::setBufferSize(unsigned int bufferSize)
{
	if(bufferSize < 1)
	{
		throw "CV7::setBufferSize: bufferSize must be 1 or higher";
	}
	m_bufferSize = bufferSize;
}

void CV7::setBaudRate(unsigned int baudRate){

	if(baudRate < 1)
	{
		throw "CV7::setBaudRate: baudRate must be 1 or higher";
	}	
	m_baudRate = baudRate;

	loadConfig(m_portName, m_baudRate);

}

void CV7::setPortName(std::string portName){

	m_portName = portName;

}

unsigned int CV7::getBufferSize()
{
	return m_bufferSize;
}

std::string CV7::refreshData()
{
	const int NON_BREAKING_SPACE = 255;
	const int BUFF_SIZE = 256;
	char buffer[BUFF_SIZE];

	int index = 0;
	while(index < BUFF_SIZE) {

		buffer[index] = serialGetchar(m_fd);
		fflush(stdout);
		
		if(NON_BREAKING_SPACE == ((int)buffer[index])) {
			std::stringstream text;
			text << "CV7::refreshData: Serial read timeout";
			throw text.str().c_str();
		}
		index++;
	}
	return buffer;
}

void CV7::parseData(std::string data) {
	if(data.size()==0) return;
	try {
		std::map<std::string, float> result = UtilityLibrary::parseString(data.c_str());
		m_windDirection.push_back(result.find("windDirection")->second);
		m_windSpeed.push_back(result.find("windSpeed")->second);
		m_windTemperature.push_back(result.find("windTemperature")->second);
	}
	catch (const char* e) {
		std::stringstream text;
		text << "CV7::parseData() [" << e << "]";
		throw text.str().c_str();
	}
	while (m_windDirection.size() > m_bufferSize) {
		m_windDirection.erase(m_windDirection.begin());
		m_windSpeed.erase(m_windSpeed.begin());
		m_windTemperature.erase(m_windTemperature.begin());
	}
}

bool CV7::isUseMean() {
	return m_useMean;
}

void CV7::setUseMean(bool useMean) {
	m_useMean = useMean;
}

float CV7::getDirection()
{
	if (m_useMean) {
		return Utility::meanOfAngles(m_windDirection);
	}
	else {
		return Utility::getMedianValue(m_windDirection);
	}
}

float CV7::getSpeed()
{
	if (m_useMean) {
		return Utility::mean(m_windSpeed);
	}
	else {
		return Utility::getMedianValue(m_windSpeed);
	}
}

float CV7::getTemperature()
{
	if (m_useMean) {
			return Utility::mean(m_windTemperature);
	}
	else {
		return Utility::getMedianValue(m_windTemperature);
	}
}

void CV7::getModel(WindsensorModel *model) {
	model->direction = getDirection();
	model->speed = getSpeed();
	model->temperature = getTemperature();
}
/*
 * GPSupdater.cpp
 *
 *  Created on: May 19, 2015
 *      Author: sailbot
 */

#include "GPSupdater.h"
#include <iostream>


GPSupdater::GPSupdater(SystemState *systemState, bool mockIt):
m_systemState(systemState),
m_running(true)
{

	if (mockIt) {
		m_gpsReader = new MockGPSReader();
	}
	else {
		m_gpsReader = new GPSReader();
	}

	try {
		m_gpsReader->connectToGPS();
	} catch (const char * error) {
		m_running=false;
		std::cout << "GPSupdater : connnectToGPS() : " << error << std::endl;
		m_logger.error(std::string("GPSupdater : connnectToGPS() : ") + error);
	}
}

void GPSupdater::run()
{
	std::cout << "*GPSupdater thread started." << std::endl;
	m_logger.info("*GPSupdater thread started.");

	while(isRunning())
	{
		try {
			m_gpsReader->readGPS(50000000); //microseconds
			m_systemState->setGPSModel(m_gpsReader->getModel());
		} catch (const char *error) {
			std::cout << "GPSupdater : readGPS() : " << error << std::endl;
			m_logger.error(std::string("GGPSupdater : readGPS() : ") + error);
		}
	}

	std::cout << "*GPSupdater thread exited." << std::endl;
	m_logger.info("*GPSupdater thread exited.");
}

void GPSupdater::close()
{
	m_mutex.lock();
	m_running = false;
	m_mutex.unlock();
}

bool GPSupdater::isRunning()
{
	bool running;
	m_mutex.lock();
	running = m_running;
	m_mutex.unlock();
	return running;
}

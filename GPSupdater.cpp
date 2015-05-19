/*
 * GPSupdater.cpp
 *
 *  Created on: May 19, 2015
 *      Author: sailbot
 */

#include "GPSupdater.h"
#include <iostream>


GPSupdater::GPSupdater(GPSReader* reader):
m_gpsReader(reader),
m_running(true)
{
	try {
		m_gpsReader->connectToGPS();
	} catch (const char * error) {
		std::cout << "GPSupdater : connnectToGPS() : " << error << std::endl;
	}
}

void GPSupdater::run()
{
	std::cout << "GPSupdater : run() : enter" << std::endl;
	while(isRunning())
	{
		std::cout << "GPSupdater : run() : exec" << std::endl;
		try {
			m_gpsReader->readGPS(50000000); //microseconds
		} catch (const char *error) {
			std::cout << "GPSupdater : readGPS() : " << error << std::endl;
		}
	}
	std::cout << "GPSupdater : run() : exit" << std::endl;
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

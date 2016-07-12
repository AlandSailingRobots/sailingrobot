/*
 * GPSupdater.cpp
 *
 *  Created on: May 19, 2015
 *      Author: sailbot
 */

#include "GPSupdater.h"
#include <iostream>
#include "gps/MockGPSReader.h"
#include "gps/GPSReader.h"


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

	if(not m_gpsReader->connectToGPS())
	{
		m_running = false;
	}
}

void GPSupdater::run()
{
	Logger::info("*GPSupdater thread started.");

	while(isRunning())
	{
		m_gpsReader->readGPS(50000000); //microseconds
		m_systemState->setGPSModel(m_gpsReader->getModel());
	}

	Logger::info("*GPSupdater thread exited.");
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

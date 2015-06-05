#include "WindsensorController.h"

#include <iostream>
#include "CV7/CV7.h"
#include "CV7/MockWindsensor.h"


WindsensorController::WindsensorController(SystemState *systemState, bool mockIt,
	std::string port_name = "non", int baud_rate = 1, int buff_size = 1):

	m_systemState(systemState),
	m_running(true)
{
	if (!mockIt) {
		m_windSensor = new CV7;
	}
	else {
		m_windSensor = new MockWindsensor;
	}

	try {
		m_windSensor->loadConfig( port_name, baud_rate );
		m_windSensor->setBufferSize( buff_size );
	} catch (const char * error) {
		//logMessage("error", error);
		throw error;
	}
	//logMessage("message", "setupWindSensor() done");
}

void WindsensorController::run()
{
	while(isRunning())
	{
		try {
			m_windSensor->parseData(m_windSensor->refreshData());	
		} catch(const char * e) {
			std::cout << "ERROR: SailingRobot::Run m_windSensor->parseData " << e << std::endl;
		}

		m_systemState->setWindsensorModel(
			WindsensorModel(
				m_windSensor->getDirection(),
				m_windSensor->getSpeed(),
				m_windSensor->getTemperature()
			)
		);
	}
}

void WindsensorController::close()
{
	m_mutex.lock();
	m_running = false;
	m_mutex.unlock();
}

bool WindsensorController::isRunning()
{
	bool running;
	m_mutex.lock();
	running = m_running;
	m_mutex.unlock();
	return running;
}

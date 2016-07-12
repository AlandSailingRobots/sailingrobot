#include "WindsensorController.h"

#include <iostream>
#include "CV7/CV7.h"
#include "CV7/MockWindsensor.h"
#include "Compass/HMC6343.h"
#include "Compass/MockCompass.h"
#include "AnalogArduino/AR_UNO.h"
#include "AnalogArduino/MockAnalogArduino.h"

WindsensorController::WindsensorController(SystemState *systemState, bool mockIt,
  	std::string port_name = "non", int baud_rate = 1,
	int buff_size = 1):

	m_systemState(systemState),
	m_running(true)
{
	if (!mockIt) {
		m_windSensor.reset(new CV7);
	}
	else {
		m_windSensor.reset(new MockWindsensor);
	}


	if(m_windSensor->loadConfig(port_name, baud_rate))
	{
		Logger::info("WindSensor setup");
	}
}

void WindsensorController::run() {
	while(isRunning())
	{

		if( m_windSensor->parseData(m_windSensor->refreshData()) )
		{
			m_systemState->setWindsensorModel(WindsensorModel( m_windSensor->getDirection(), m_windSensor->getSpeed(),
										  m_windSensor->getTemperature() ));
		}

		//TODO - Jordan: Might want to do something if we keep failing to parse wind data.
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

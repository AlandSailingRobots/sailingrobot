#include "WindsensorController.h"

#include <iostream>
#include "CV7/CV7.h"
#include "CV7/MockWindsensor.h"
#include "Compass/HMC6343.h"
#include "Compass/MockCompass.h"
#include "AnalogArduino/PTMN_STS.h"
#include "AnalogArduino/MockAnalogArduino.h"

WindsensorController::WindsensorController(SystemState *systemState, bool mockIt, bool mockCompass,
	bool mockPressure, std::string port_name = "non", int baud_rate = 1,
	int buff_size = 1, int headningBufferSize = 1):

	m_systemState(systemState),
	m_running(true)
{
	if (!mockIt) {
		m_windSensor.reset(new CV7);
	}
	else {
		m_windSensor.reset(new MockWindsensor);
	}

	if(!mockPressure) {
		m_pressure.reset(new PTMN_STS());
	} else {
		m_pressure.reset(new MockAnalogArduino());
	}

	try {
		m_windSensor->loadConfig( port_name, baud_rate );
		m_windSensor->setBufferSize( buff_size );
	} catch (const char * error) {
		m_logger.error(error);
		throw error;
	}
	m_logger.info("setupWindSensor() done\t\t");

    m_logger.info("Starting Compass\t\t");
    initCompass(mockCompass,headningBufferSize);
}

void WindsensorController::run() {
	while(isRunning()){
        m_compass->readValues();
        m_compass->readAccel();

        //update system state
		m_systemState->setCompassModel(CompassModel(
		m_compass->getHeading(),
		m_compass->getPitch(),
		m_compass->getRoll(),
        AccelerationModel(m_compass->getAccelX(),
                          m_compass->getAccelY(),
                          m_compass->getAccelZ() )
		));

		try {
			m_windSensor->parseData(m_windSensor->refreshData());
		} catch(const char * e) {
			std::cout << "ERROR: SailingRobot::Run m_windSensor->parseData " << e << std::endl;
			m_logger.error(std::string("WindsensorController::Run m_windSensor->parseData ") + e);
		}

		m_systemState->setWindsensorModel(WindsensorModel(
										  m_windSensor->getDirection(),
										  m_windSensor->getSpeed(),
										  m_windSensor->getTemperature() )
		);

		m_systemState->setPressure(m_pressure->getValue());
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


 void WindsensorController::initCompass(bool mockCompass,int headningBufferSize) {
    if (!mockCompass) {
        m_compass.reset(new HMC6343(headningBufferSize) );
    } else {
        m_compass.reset(new MockCompass);
    }

    try {
        m_compass->init();
    } catch (const char * error) {
        m_logger.error("SailingRobot::setupCompass() failed");
    }

    m_logger.info("setupCompass() done\t\t");
 }

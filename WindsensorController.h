#ifndef __WINDSENSORCONTROLLER_H__
#define __WINDSENSORCONTROLLER_H__

#include <memory>
#include <mutex>

#include "CV7/Windsensor.h"
#include "thread/SystemState.h"
#include "logger/Logger.h"
#include "Compass/Compass.h"
#include "AnalogArduino/AnalogArduino.h"
#include "AnalogArduino/MockAnalogArduino.h"

class WindsensorController
{
public:

	WindsensorController(SystemState *systemState, bool mockIt, 
		std::string port_name, int baud_rate,
		int buff_size);
	~WindsensorController() {};

	void run();
	void close();

private:

	std::unique_ptr<Windsensor> m_windSensor;
	SystemState* m_systemState;

	std::mutex m_mutex;
	bool m_running;

    std::unique_ptr<Compass> m_compass;
	std::unique_ptr<AnalogArduino> m_pressure;

	Logger m_logger;

	bool isRunning();
};

#endif

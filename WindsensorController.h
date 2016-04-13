#ifndef __WINDSENSORCONTROLLER_H__
#define __WINDSENSORCONTROLLER_H__

#include <mutex>

#include "CV7/Windsensor.h"
#include "thread/SystemState.h"
#include "logger/Logger.h"
#include "Compass/Compass.h"

class WindsensorController
{
public:

	WindsensorController(SystemState *systemState, bool mockIt, bool mockCompass,
		std::string port_name, int baud_rate, int buff_size, int headningBufferSize);
	~WindsensorController() {};

	void run();
	void close();
	
private:

	Windsensor* m_windSensor;
	SystemState* m_systemState;

	std::mutex m_mutex;
	bool m_running;
        
        Compass* m_compass;

	Logger m_logger;

	bool isRunning();
        
        void initCompass(bool mockCompass,int headningBufferSize);
};

#endif

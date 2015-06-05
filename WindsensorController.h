#ifndef __WINDSENSORCONTROLLER_H__
#define __WINDSENSORCONTROLLER_H__

#include <mutex>

#include "CV7/Windsensor.h"
#include "thread/SystemState.h"

class WindsensorController
{
public:

	WindsensorController(SystemState *systemState, bool mockIt,
		std::string port_name, int baud_rate, int buff_size);
	~WindsensorController() {};

	void run();
	void close();
	
private:

	Windsensor* m_windSensor;
	SystemState* m_systemState;

	std::mutex m_mutex;
	bool m_running;

	bool isRunning();
};

#endif

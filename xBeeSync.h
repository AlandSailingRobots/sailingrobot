#ifndef __XBEESYNC_H__
#define __XBEESYNC_H__

#include "xBee/xBee.h"
#include "xmlparser/src/xml_log.h"
#include "thread/SystemState.h"
#include "models/SystemStateModel.h"
#include <mutex>

class xBeeSync
{
public:
	xBeeSync(SystemState *systemState);
	~xBeeSync() {};

	void run();
	void close();
	
private:
	XML_log m_XML_log;
	xBee m_xBee;
	int m_xbee_fd;
	SystemStateModel m_model;
	SystemState *m_system_state;

	std::mutex m_mutex;
	bool m_running;

	bool isRunning();
};

#endif
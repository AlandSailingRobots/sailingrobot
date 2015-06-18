#ifndef __XBEESYNC_H__
#define __XBEESYNC_H__

#include "xBee/xBee.h"
#include "xmlparser/src/xml_log.h"
#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"
#include "models/SystemStateModel.h"
#include "logger/Logger.h"
#include <mutex>

class xBeeSync
{
public:
	xBeeSync(ExternalCommand* externalCommand, SystemState *systemState,
			 bool sending, bool receiving);
	~xBeeSync() {};

	void run();
	void close();
	
private:
	XML_log m_XML_log;
	xBee m_xBee;
	int m_xbee_fd;
	ExternalCommand* m_external_command;
	SystemStateModel m_model;
	SystemState *m_system_state;

	std::mutex m_mutex;
	bool m_running;
	bool m_sending;
	bool m_receiving;
	Logger m_logger;

	bool isRunning();
};

#endif

#ifndef __XBEESYNC_H__
#define __XBEESYNC_H__

#include "xBee/xBee.h"
#include "xmlparser/src/xml_log.h"
#include "thread/ExternalCommand.h"
#include "thread/SystemState.h"
#include "models/SystemStateModel.h"
#include "logger/Logger.h"
#include "dbhandler/DBHandler.h"
#include "utility/Timer.h"
#include <mutex>

class xBeeSyncNode : public ActiveNode
{
public:
	xBeeSyncNode(DBHandler* db, bool sendLogs, bool sending, bool receiving, double loopTime);
	~xBeeSyncNode() {};

	/////////////////////////////////////////////////////////////////////////////////////
 	/// Initialises the xBee module and returns false if the function fails to setup the
 	/// xbee. 
 	///
 	/////////////////////////////////////////////////////////////////////////////////////
	bool init();
	void start();

	void xBeeSyncThread();
	
private:
	XML_log m_XML_log;
	xBee m_xBee;
	int m_xbee_fd;
	SystemStateModel m_model;
	DBHandler *m_db;
	Timer m_timer;

	std::mutex m_mutex;
	bool m_running;
	bool m_sending;
	bool m_receiving;
	bool m_sendLogs;
	bool m_pushOnlyLatestLogs;
	bool m_initialised;
	double m_loopTime;

	bool isRunning();
};

#endif

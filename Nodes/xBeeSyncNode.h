
/****************************************************************************************
 *
 * File:
 * 		xBeeSyncNode.h
 *
 * Purpose:
 *		Handles sending of logs or the current system state (xml-format) over xbee,
 * 		depending on the setting of send_logs. Also receives rudder and sail control messages
 *		over the xbee network.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

 #pragma once

#include "ActiveNode.h"
#include "xBee/xBee.h"
#include "xmlparser/src/xml_log.h"
#include "logger/Logger.h"
#include "dbhandler/DBHandler.h"
#include "utility/Timer.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/ActuatorPositionMsg.h"

class xBeeSyncNode : public ActiveNode {
public:
	xBeeSyncNode(MessageBus& msgBus, DBHandler& db);

	///----------------------------------------------------------------------------------
	/// Gets all settings from database and initializes xbee object
	///
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
	/// Starts the thread if init is done
	///
	///----------------------------------------------------------------------------------
	void start();
	void processMessage(const Message*);
	
private:

	///----------------------------------------------------------------------------------
	/// Sends an xml-formated vessel state string over the xbee network
	///
	///----------------------------------------------------------------------------------
	void sendVesselState(VesselStateMsg* msg);

	///----------------------------------------------------------------------------------
	/// Sends a log string over the xbee network
	///
	///----------------------------------------------------------------------------------
	void sendLogs();

	///----------------------------------------------------------------------------------
	/// Generates an actuator control message from data received by xbee
	///
	///----------------------------------------------------------------------------------
	void receiveControl();

	///----------------------------------------------------------------------------------
	/// Polls receiveControl and sendlogs repeatedly
	///
	///----------------------------------------------------------------------------------
	static void xBeeSyncThread(void* nodePtr);

	XML_log m_XML_log;
	xBee m_xBee;
	int m_xbee_fd;
	DBHandler &m_db;
	Timer m_timer;

	bool m_running;
	bool m_sending;
	bool m_receiving;
	bool m_sendLogs;
	bool m_pushOnlyLatestLogs;
	bool m_initialised;
	double m_loopTime;
	int m_messageTimeBuffer;

};



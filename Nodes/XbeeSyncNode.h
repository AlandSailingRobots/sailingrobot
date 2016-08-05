
/****************************************************************************************
 *
 * File:
 * 		XbeeSyncNode.h
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
#include "xBee/Xbee.h"
#include "SystemServices/Logger.h"
#include "dbhandler/DBHandler.h"
#include "Messages/VesselStateMsg.h"

class XbeeSyncNode : public ActiveNode {
public:
	XbeeSyncNode(MessageBus& msgBus, DBHandler& db);
	virtual ~XbeeSyncNode(){}

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
	/// Serialises a vessel state message and passes it to the Xbee for sending.
	///----------------------------------------------------------------------------------
	void sendMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Called when a message has been received by the Xbee radio.
	///----------------------------------------------------------------------------------
	static void incomingMessage(uint8_t* data, uint8_t size);

	///----------------------------------------------------------------------------------
	/// Transmits and receives messages from the xbee.
	///----------------------------------------------------------------------------------
	static void xBeeSyncThread(void* nodePtr);

	Xbee m_xbee;
	bool m_initialised;
	DBHandler &m_db;
	static XbeeSyncNode* m_node;

	bool m_running;
	bool m_sending;
	bool m_receiving;
	bool m_sendLogs;
	bool m_pushOnlyLatestLogs;
	double m_loopTime;
	double m_lastMessageCallTime;
	bool m_firstMessageCall;

};



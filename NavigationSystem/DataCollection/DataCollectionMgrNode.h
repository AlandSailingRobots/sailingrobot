/****************************************************************************************
*
* File:
* 		DataCollectionMgrNode.h
*
* Purpose:
*		Listen to WayPointDataMsg and LocalConfigChangeMsg,
*		send DataCollectionStartMsg, DataCollectionStopMsg and DataRequestMsg
*		accordingly.
*
* Developer Notes:
*
*
***************************************************************************************/


#pragma once

#include "MessageBus/MessageBus.h"

class DataCollectionMgrNode : public Node {
public:

	DataCollectionMgrNode(MessageBus& msgBus, DBHandler& db);

	///----------------------------------------------------------------------------------
	/// Process the message.
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Read the data.
	///----------------------------------------------------------------------------------
	bool readConfig(int& timeInterval, bool& measureAtCheckpoint);

private:

	void sendMessage();

	int		m_timeInterval;
	bool	m_measureAtCheckpoint;
	bool	m_isCheckpoint
	DBHandler &m_db;

};

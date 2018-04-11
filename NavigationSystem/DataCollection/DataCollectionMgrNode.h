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

	DataCollectionMgrNode(MessageBus& msgBus);

	///----------------------------------------------------------------------------------
	/// Process the message.
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Read the new config data.
	///----------------------------------------------------------------------------------
	bool readConfig(int& timeInterval, bool& measureAtCheckpoint);

private:

	void sendIntervalMessage();
	void sendRequestMessage();

	int		m_timeInterval;
	bool	m_measureAtCheckpoint;
	bool	m_isCheckpoint

};

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
	/// Read the data.
	///----------------------------------------------------------------------------------
	bool readData(int& timeInterval, bool& measureAtCheckpoint);

private:

	void sendStartMessage();
	void sendStopMessage();
	void sendRequestMsg();

	int		m_timeInterval;
	bool	m_measureAtCheckpoint;

};

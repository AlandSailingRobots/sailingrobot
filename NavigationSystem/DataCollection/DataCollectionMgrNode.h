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
*
***************************************************************************************/


#pragma once

#include "MessageBus/MessageBus.h"

class DataCollectionMgrNode : public Node {
public:

	///----------------------------------------------------------------------------------
	/// Process the data requsest message.
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msg);

private:
	
};

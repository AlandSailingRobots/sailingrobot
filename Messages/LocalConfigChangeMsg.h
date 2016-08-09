
/****************************************************************************************
 * Purpose:
 *		Notify receivers that the configuration settings in the local database have changed
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"

class LocalConfigChangeMsg : public Message {
public:
	LocalConfigChangeMsg(NodeID destinationID, NodeID sourceID)
		:Message(MessageType::LocalConfigChange, sourceID, destinationID)
	{ }

	LocalConfigChangeMsg()
		:Message(MessageType::LocalConfigChange, NodeID::None, NodeID::None)
	{ }

	LocalConfigChangeMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{ }

	virtual ~LocalConfigChangeMsg() { }

};


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

	virtual ~LocalConfigChangeMsg() { }

};

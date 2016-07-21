
#pragma once

#include "Message.h"

class ServerConfigsReceivedMsg : public Message {
public:
	ServerConfigsReceivedMsg(NodeID destinationID, NodeID sourceID)
		:Message(MessageType::ServerConfigsReceived, sourceID, destinationID)
	{ }

	ServerConfigsReceivedMsg()
		:Message(MessageType::ServerConfigsReceived, NodeID::None, NodeID::None)
	{ }

	virtual ~ServerConfigsReceivedMsg() { }

};

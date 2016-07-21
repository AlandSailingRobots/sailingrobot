
#pragma once

#include "Message.h"

class ServerConfigsReceivedMsg : public Message {
public:
	ServerConfigsReceivedMsg(NodeID destinationID, NodeID sourceID)
		:Message(MessageType::ServerConfigReceived, sourceID, destinationID)
	{ }

	ServerConfigsReceivedMsg()
		:Message(MessageType::ServerConfigReceived, NodeID::None, NodeID::None)
	{ }

	virtual ~ServerConfigsReceivedMsg() { }

};

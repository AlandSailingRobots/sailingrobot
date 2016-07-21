
#pragma once

#include "Message.h"

class ServerWaypointsReceivedMsg : public Message {
public:
	ServerWaypointsReceivedMsg(NodeID destinationID, NodeID sourceID)
		:Message(MessageType::ServerWaypointsReceived, sourceID, destinationID)
	{ }

	ServerWaypointsReceivedMsg()
		:Message(MessageType::ServerWaypointsReceived, NodeID::None, NodeID::None)
	{ }

	virtual ~ServerWaypointsReceivedMsg() { }

};

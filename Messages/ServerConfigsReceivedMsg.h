
/****************************************************************************************
 * Purpose:
 *		Notify receivers that new config settings have been downloaded from the server
 *
 * Developer Notes:
 *
 ***************************************************************************************/

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

	ServerConfigsReceivedMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{ }

	virtual ~ServerConfigsReceivedMsg() { }

};

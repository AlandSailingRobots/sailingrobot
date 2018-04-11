/****************************************************************************************
 *
 * File:
 * 		DataRequestMsg.h
 *
 * Purpose:
 *		A DataRequestMsg is used to instruct a node to offer up any data it is storing
 *		the message bus.
 *
 * Developer Notes:
 *		If the source id of this message is set, it should be used as
 *		the returen address, otherwise post the returning data message to the message
 *		bus for general consumption.
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"


class DataRequestMsg : public Message {
public:
	DataRequestMsg(NodeID destinationID, NodeID sourceID)
		:Message(MessageType::DataRequest, sourceID, destinationID) { }

	DataRequestMsg(NodeID destinationID)
		:Message(MessageType::DataRequest, NodeID::None, destinationID) { }

	DataRequestMsg()
		:Message(MessageType::DataRequest, NodeID::None, NodeID::None) { }

	DataRequestMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{ }

	virtual ~DataRequestMsg() { }
};

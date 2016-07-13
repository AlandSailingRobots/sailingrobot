/****************************************************************************************
 *
 * File:
 * 		Node.h
 *
 * Purpose:
 *		The base passive node.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

class MessageBus;

#include "NodeIDs.h"
#include "MessageBus.h"
#include "Messages/Message.h"

class Node {
public:
	Node(NodeID id, MessageBus& msgBus) : m_MsgBus(msgBus), m_NodeID(id)
	{ }


	NodeID nodeID() { return m_NodeID; }

	virtual void processMessage(Message* message) = 0;
protected:
	MessageBus& m_MsgBus;
private:
	const NodeID m_NodeID;
};
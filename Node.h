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

	virtual ~Node(){};

	///----------------------------------------------------------------------------------
 	/// This function should attempt to initialise or setup any resources it may need to
 	/// function. If this was successful this function should return true.
 	///
 	///----------------------------------------------------------------------------------
	virtual bool init() = 0;

	///----------------------------------------------------------------------------------
 	/// Called by the MessageBus when it has a message the node might be interested in.
 	/// A node should register for messages it wants to receive using 
 	/// MessageBus::registerNode(Node*, MessageType)
 	///
 	///----------------------------------------------------------------------------------
	virtual void processMessage(const Message* message) = 0;

	NodeID nodeID() { return m_NodeID; }
protected:
	MessageBus& m_MsgBus;
private:
	NodeID m_NodeID;
};
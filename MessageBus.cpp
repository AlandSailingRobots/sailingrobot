/****************************************************************************************
 *
 * File:
 * 		MessageBus.cpp
 *
 * Purpose:
 *		The message bus manages message distribution to nodes allowing nodes to
 *		communicate with one another.
 *
 ***************************************************************************************/

#include "MessageBus.h"


MessageBus::MessageBus()
	:m_Running(false)
{

}

MessageBus::~MessageBus()
{
	for (auto it = m_RegisteredNodes.begin(); it != m_RegisteredNodes.end(); ++it) {
    	delete *it;
	}
}

void MessageBus::registerNode(Node* node)
{
	if(not m_Running)
	{
		// Don't care about the return type, only want to register the node.
		getRegisteredNode(node);
	}
}

void MessageBus::registerNode(Node* node, MessageType msgType)
{
	if(not m_Running)
	{
		RegisteredNode* regNode = getRegisteredNode(node);
		regNode->subscribe(msgType);
	}
}

void MessageBus::sendMessage(Message* msg)
{

}

void MessageBus::run()
{
	// Prevent nodes from being registered now
	m_Running = true;

	while(m_Running)
	{

	}
}

MessageBus::RegisteredNode* MessageBus::getRegisteredNode(Node* node)
{
	for(auto regNode : m_RegisteredNodes)
	{
		if(regNode->nodePtr == node)
		{
			return regNode;
		}
	}

	RegisteredNode* newRegNode = new RegisteredNode(node);
	m_RegisteredNodes.push_back(newRegNode);
	return newRegNode;
}
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
#include "logger/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>


#define SLEEP_TIME_MS	50


MessageBus::MessageBus()
	:m_Running(false)
{
	m_FrontMessages = new std::queue<Message*>();
	m_BackMessages = new std::queue<Message*>();
}

MessageBus::~MessageBus()
{
	for (auto it = m_RegisteredNodes.begin(); it != m_RegisteredNodes.end(); ++it) {
    	delete *it;
	}

	delete m_FrontMessages;
	delete m_BackMessages;
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
	if(msg != NULL)
	{
		m_FrontQueueMutex.lock();
		m_FrontMessages->push(msg);
		m_FrontQueueMutex.unlock();
	}
}

void MessageBus::run()
{
	// Prevent nodes from being registered now
	m_Running = true;

	while(m_Running)
	{
		// TODO - Jordan: Wake up only when messages have been pushed into the queue.

		std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));

		// If we have messages, flip the two queues and begin processing messages
		if(m_FrontMessages->size() > 0)
		{
			std::queue<Message*>* tmpPtr;

			m_FrontQueueMutex.lock();

			tmpPtr = m_FrontMessages;
			m_FrontMessages = m_BackMessages;

			m_FrontQueueMutex.unlock();

			m_BackMessages = tmpPtr;

			processMessages();
		}
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

	Logger::info("New node registered");
	RegisteredNode* newRegNode = new RegisteredNode(node);
	m_RegisteredNodes.push_back(newRegNode);
	return newRegNode;
}

void MessageBus::processMessages()
{
	while(m_BackMessages->size() > 0)
	{
		Message* msg = m_BackMessages->front();

		for(auto node : m_RegisteredNodes)
		{
			// Distribute to everyone interested
			if(msg->destinationID() == NodeID::None)
			{
				if(node->isInterested( msg->messageType() ))
				{
					node->nodePtr->processMessage(msg);
				}
			}
			// Distribute to the node the message is directed at then move onto the next message
			else
			{
				if(node->nodePtr->nodeID() == msg->destinationID())
				{
					node->nodePtr->processMessage(msg);
					continue;
				}
			}
		}

		m_BackMessages->pop();
		delete msg;
	}
}

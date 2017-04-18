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
#include "SystemServices/Logger.h"
#include <sys/time.h>
#include "SystemServices/SysClock.h"

// For std::this_thread
#include <chrono>
#include <thread>
#include <iostream>

#define SLEEP_TIME_MS	50


MessageBus::MessageBus()
	:m_Running(false)
{
	m_FrontMessages = new std::queue<MessagePtr>();
	m_BackMessages = new std::queue<MessagePtr>();
}

MessageBus::~MessageBus()
{
	for (auto it = m_RegisteredNodes.begin(); it != m_RegisteredNodes.end(); ++it) {
    	delete *it;
	}

	delete m_FrontMessages;
	delete m_BackMessages;

	std::cout << "Message Bus Destructor" << std::endl;
}

// TODO - Jordan: Log warning if node tries to register after start

bool MessageBus::registerNode(Node& node)

{
	if(not m_Running)
	{
		// Don't care about the return type, only want to register the node.
		getRegisteredNode(node);
		return true;
	}
	return false;
}

bool MessageBus::registerNode(Node& node, MessageType msgType)

{
	if(not m_Running)
	{
		RegisteredNode* regNode = getRegisteredNode(node);
		regNode->subscribe(msgType);
		return true;
	}
	return false;
}

void MessageBus::sendMessage(MessagePtr msg)
{
	if(msg != NULL)
	{
		m_FrontQueueMutex.lock();
		Message* logMsg = msg.get();
		m_FrontMessages->push(std::move(msg));
		logMessageReceived(logMsg); 
		m_FrontQueueMutex.unlock();
	}
}

void MessageBus::run()
{
	// Prevent nodes from being registered now
	m_Running = true;

	startMessageLog();

	while(m_Running)
	{
		// TODO - Jordan: Wake up only when messages have been pushed into the queue.

		std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));

		// If we have messages, flip the two queues and begin processing messages
		if(m_FrontMessages->size() > 0)
		{
			std::queue<MessagePtr>* tmpPtr;

			m_FrontQueueMutex.lock();

			tmpPtr = m_FrontMessages;
			m_FrontMessages = m_BackMessages;

			m_FrontQueueMutex.unlock();

			m_BackMessages = tmpPtr;

			processMessages();
		}
	}
}

//TODO - Jordan: What would cause this to return a null pointer?
MessageBus::RegisteredNode* MessageBus::getRegisteredNode(Node& node)

{
	for(auto regNode : m_RegisteredNodes)
	{
		if(regNode->nodeRef.nodeID() == node.nodeID())
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

		MessagePtr msgPtr = std::move(m_BackMessages->front());
		Message* msg = msgPtr.get();

		logMessage(msg);

		for(auto node : m_RegisteredNodes)
		{
			// Distribute to everyone interested
			if(msg->destinationID() == NodeID::None)
			{
				if(node->isInterested( msg->messageType() ))
				{
					node->nodeRef.processMessage(msg);
					logMessageConsumer(node->nodeRef.nodeID());

				}
			}
			// Distribute to the node the message is directed at then move onto the next message
			else
			{
				if(node->nodeRef.nodeID() == msg->destinationID())
				{
					node->nodeRef.processMessage(msg);
					logMessageConsumer(node->nodeRef.nodeID());

					continue;
				}
			}
		}

		m_BackMessages->pop();

		// delete msg; Don't need for unique pointers
	}

}

void MessageBus::startMessageLog()
{
#ifdef LOG_MESSAGES
	m_LogFile = new std::ofstream("./Messages.log", std::ios::out | std::ios::trunc);
	if(m_LogFile->is_open())
	{
		Logger::info("Message log file created!");
	}
	else
	{
		Logger::error("Message log file not created!");
		delete m_LogFile;
		m_LogFile = NULL;
	}
#endif
}

void MessageBus::logMessageReceived(Message* msg)
{
#ifdef LOG_MESSAGES
	msg->timeReceived = SysClock::timeStamp();
#endif
}

void MessageBus::logMessage(Message* msg)
{
#ifdef LOG_MESSAGES
	if(m_LogFile != NULL)
	{
		char buff[256];

		snprintf(buff, 256, "[%s] Type=%s(%d) SourceID=%d Destination=%d Received=%s", 	SysClock::hh_mm_ss_ms().c_str(),
																						msgToString(msg->messageType()).c_str(),
																						(int) msg->messageType(), (int) msg->sourceID(),
																						(int) msg->destinationID(),
																						SysClock::hh_mm_ss_ms(msg->timeReceived).c_str());
		*m_LogFile << buff << "\n";
		m_LogFile->flush();
	}
#endif
}

void MessageBus::logMessageConsumer(NodeID id)
{
#ifdef LOG_MESSAGES
	if(m_LogFile != NULL)
	{
		char buff[256];

		snprintf(buff, 256, "\t%s Consumed by Node: %s(%d)", SysClock::hh_mm_ss_ms().c_str(), nodeToString(id).c_str(), (int) id);
		*m_LogFile << buff << "\n";
		m_LogFile->flush();
	}
#endif
}

void MessageBus::messageTimeStamp(unsigned long unixTime, char* buffer)
{
	char buff[9];
	time_t unix_time = (time_t)unixTime;
	strftime(buff, 9, "%H:%M:%S", gmtime(&unix_time));

	// Get Milliseconds
	timeval curTime;
	gettimeofday(&curTime, NULL);
	int milli = curTime.tv_usec / 1000;

	sprintf(buffer, "%s:%d", buff, milli);

}

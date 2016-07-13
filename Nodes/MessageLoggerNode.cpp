/****************************************************************************************
 *
 * File:
 * 		MessageLoggerNode.cpp
 *
 * Purpose:
 *		The message logger node, logs all message that come through the system.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "MessageLoggerNode.h"
#include "logger/Logger.h"


//TODO - Jordan: File logging


MessageLoggerNode::MessageLoggerNode(MessageBus& msgBus)
	:Node(NodeID::MessageLogger, msgBus)
{
	msgBus.registerNode(this);
}

bool MessageLoggerNode::init()
{
	return true;
}

void MessageLoggerNode::processMessage(const Message* message)
{
	MessageType msgType = message->messageType();

	switch(msgType)
	{
		case MessageType::DataRequest:
			Logger::info("DataRequest message received");
			break;
	}
}
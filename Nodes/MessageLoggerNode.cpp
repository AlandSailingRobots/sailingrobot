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
	msgBus.registerNode(this, MessageType::WindData);
	msgBus.registerNode(this, MessageType::CompassData);
	msgBus.registerNode(this, MessageType::GPSData);
	msgBus.registerNode(this, MessageType::ArduinoData);
	msgBus.registerNode(this, MessageType::WaypointData);
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
		case MessageType::WindData:
			Logger::info("WindData message received");
			break;
		case MessageType::CompassData:
			Logger::info("CompassData message received");
			break;
		case MessageType::GPSData:
			Logger::info("GPSData message received");
			break;
		case MessageType::ArduinoData:
			Logger::info("ArduinoData message received");
			break;
		case MessageType::WaypointData:
			Logger::info("WaypointData message received");
			break;
		default:
			break;
	}
}

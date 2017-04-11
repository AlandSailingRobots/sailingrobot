/****************************************************************************************
 *
 * File:
 * 		MessageLogger.h
 *
 * Purpose:
 *		Catches all known message types.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Nodes/Node.h"


class MessageLogger: public Node {
public:
	MessageLogger(MessageBus& msgBus)
		:Node(NodeID::MessageLogger, msgBus), m_DataRequest(false), m_WindData(false), m_CompassData(false), m_GPSData(false), m_WindState(false)
	{
		msgBus.registerNode(*this);
		msgBus.registerNode(*this, MessageType::WindData);
		msgBus.registerNode(*this, MessageType::CompassData);
		msgBus.registerNode(*this, MessageType::GPSData);
		msgBus.registerNode(*this, MessageType::WaypointData);
		msgBus.registerNode(*this, MessageType::ActuatorPosition);
		msgBus.registerNode(*this, MessageType::WindState);
	}

	virtual ~MessageLogger() { }

	bool init() { return true; }

	void processMessage(const Message* message)
	{
		MessageType msgType = message->messageType();

		switch(msgType)
		{
			case MessageType::DataRequest:
				m_DataRequest = true;
				break;
			case MessageType::WindData:
				m_WindData = true;
				break;
			case MessageType::CompassData:
				m_CompassData = true;
				break;
			case MessageType::GPSData:
				m_GPSData = true;
				break;
			case MessageType::WaypointData:
				m_waypointData = true;
				break;
			case MessageType::ActuatorPosition:
				m_actuatorPosition = true;
				break;
			case MessageType::WindState:
				m_WindState = true;
				break;
			default:
				return;
		}
	}


	bool dataRequestReceived() { return m_DataRequest; }
	bool windDataReceived() { return m_WindData; }
	bool compassDataReceived() { return m_CompassData; }
	bool gpsDataReceived() { return m_GPSData; }
	bool waypointDataReceived() { return m_waypointData; }
	bool actuatorPositionReceived() { return m_actuatorPosition; }
	bool windStateReceived() { return m_WindState; }

private:
	bool m_DataRequest;
	bool m_WindData;
	bool m_CompassData;
	bool m_GPSData;
	bool m_waypointData;
	bool m_actuatorPosition;
	bool m_WindState;
};

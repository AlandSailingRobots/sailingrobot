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

#include "MessageBus/Node.h"


class MessageLogger: public Node {
public:
	MessageLogger(MessageBus& msgBus)
	:Node(NodeID::MessageLogger, msgBus), m_DataRequest(false), m_WindData(false),
	m_CompassData(false), m_GPSData(false), m_waypointData(false),
	m_actuatorPosition(false),m_WindState(false), m_StateData(false),
	m_NavigationData(false)
	{
		msgBus.registerNode(*this);
		msgBus.registerNode(*this, MessageType::WindData);
		msgBus.registerNode(*this, MessageType::CompassData);
		msgBus.registerNode(*this, MessageType::GPSData);
		msgBus.registerNode(*this, MessageType::WaypointData);
		msgBus.registerNode(*this, MessageType::ActuatorPosition);
		msgBus.registerNode(*this, MessageType::WindState);
		msgBus.registerNode(*this, MessageType::StateMessage);
		msgBus.registerNode(*this, MessageType::NavigationControl);
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
			case MessageType::StateMessage:
			m_StateData = true;
			break;
			case MessageType::NavigationControl:
			m_NavigationData = true;
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
	bool actuatorPositionReceived() {return m_actuatorPosition; }
	bool windStateReceived() { return m_WindState; }
	bool stateDataReceived() { return m_StateData; }
	bool navigationDataReceived(){return m_NavigationData;}

	void clearState(){
		m_DataRequest = false;
		m_WindData = false;
		m_CompassData = false;
		m_GPSData = false;
		m_waypointData = false;
		m_actuatorPosition = false;
		m_WindState = false;
		m_StateData = false;
	}


private:
	bool m_DataRequest;
	bool m_WindData;
	bool m_CompassData;
	bool m_GPSData;
	bool m_waypointData;
	bool m_actuatorPosition;
	bool m_WindState;
	bool m_StateData;
	bool m_NavigationData;
};

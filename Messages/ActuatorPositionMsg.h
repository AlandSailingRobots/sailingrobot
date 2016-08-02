/****************************************************************************************
 *
 * File:
 * 		ActuatorPositionMsg.h
 *
 * Purpose:
 *		A message contain an actuator's position.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class ActuatorPositionMsg : public Message {
public:
	ActuatorPositionMsg(NodeID destinationID, NodeID sourceID, int rudderPosition, int sailPosition)
		:Message(MessageType::ActuatorPosition, sourceID, destinationID), m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	ActuatorPositionMsg(int rudderPosition, int sailPosition)
		:Message(MessageType::ActuatorPosition, NodeID::None, NodeID::None), m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	virtual ~ActuatorPositionMsg() { }

	int sailPosition() { return m_sailPosition; }
	int rudderPosition() { return m_rudderPosition; }

private:
	int m_rudderPosition;
	int m_sailPosition;
};

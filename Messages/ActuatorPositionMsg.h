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
	ActuatorPositionMsg(NodeID destinationID, NodeID sourceID, int position)
		:Message(MessageType::ActuatorPosition, sourceID, destinationID), m_Position(position)
	{ }

	ActuatorPositionMsg(int position)
		:Message(MessageType::ActuatorPosition, NodeID::None, NodeID::None), m_Position(position)
	{ }

	virtual ~ActuatorPositionMsg() { }

	int position() { return m_Position; }

private:
	int m_Position;
};

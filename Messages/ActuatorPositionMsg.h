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
	ActuatorPositionMsg(NodeID destinationID, NodeID sourceID, uint16_t rudderPosition, uint16_t sailPosition)
		:Message(MessageType::ActuatorCommand, sourceID, destinationID), m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	ActuatorPositionMsg(uint16_t rudderPosition, uint16_t sailPosition)
		:Message(MessageType::ActuatorCommand, NodeID::None, NodeID::None), m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	ActuatorPositionMsg(NodeID destinationID, NodeID sourceID, uint16_t rudderPosition, uint16_t sailPosition, bool isFeedback)
		:Message(isFeedback ? MessageType::ActuatorFeedback : MessageType::ActuatorCommand, sourceID, destinationID),
			m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	ActuatorPositionMsg(uint16_t rudderPosition, uint16_t sailPosition, bool isFeedback)
		:Message(isFeedback ? MessageType::ActuatorFeedback : MessageType::ActuatorCommand, NodeID::None, NodeID::None),
			m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	virtual ~ActuatorPositionMsg() { }

	uint16_t sailPosition() { return m_sailPosition; }
	uint16_t rudderPosition() { return m_rudderPosition; }

private:
	uint16_t m_rudderPosition;
	uint16_t m_sailPosition;
};

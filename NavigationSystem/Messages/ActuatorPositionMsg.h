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

#include "MessageBus/Message.h"


class ActuatorPositionMsg : public Message {
public:
	ActuatorPositionMsg(NodeID destinationID, NodeID sourceID, int rudderPosition, int sailPosition)
		:Message(MessageType::ActuatorPosition, sourceID, destinationID), m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	ActuatorPositionMsg(int rudderPosition, int sailPosition)
		:Message(MessageType::ActuatorPosition, NodeID::None, NodeID::None), m_rudderPosition(rudderPosition), m_sailPosition(sailPosition)
	{ }

	ActuatorPositionMsg(MessageDeserialiser deserialiser)
			:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_rudderPosition) ||
			!deserialiser.readInt(m_sailPosition))
		{
			m_valid = false;
		}
	}

	virtual ~ActuatorPositionMsg() { }

	int sailPosition() const { return m_sailPosition; }
	int rudderPosition() const { return m_rudderPosition; }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_rudderPosition);
		serialiser.serialise(m_sailPosition);
	}

private:
	int m_rudderPosition;
	int m_sailPosition;
};

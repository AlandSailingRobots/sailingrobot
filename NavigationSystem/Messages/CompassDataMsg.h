/****************************************************************************************
 *
 * File:
 * 		CompassDataMsg.h
 *
 * Purpose:
 *		A CompassDataMsg contains compass data such as heading, pitch, and roll.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"


class CompassDataMsg : public Message {
public:
	CompassDataMsg(NodeID destinationID, NodeID sourceID, int heading, int pitch, int roll)
		:Message(MessageType::CompassData, sourceID, destinationID), m_Heading(heading), m_Pitch(pitch), m_Roll(roll)
	{ }

	CompassDataMsg(int heading, int pitch, int roll)
		:Message(MessageType::CompassData, NodeID::None, NodeID::None), m_Heading(heading), m_Pitch(pitch), m_Roll(roll)
	{ }

	CompassDataMsg(MessageDeserialiser& deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_Heading) ||
			!deserialiser.readInt(m_Pitch) ||
			!deserialiser.readInt(m_Roll))
		{
			m_valid = false;
		}
	}

	virtual ~CompassDataMsg() { }

	int heading() const { return m_Heading; }
	int pitch() const { return m_Pitch; }
	int roll() const { return m_Roll; }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_Heading);
		serialiser.serialise(m_Pitch);
		serialiser.serialise(m_Roll);
	}

private:
	int m_Heading;		// degree [0, 360[ in North-East-Down reference frame
	int m_Pitch;		// degree ]-90, +90] in North-East-Down reference frame
	int m_Roll;			// degree ]-180, +180] in North-East-Down reference frame
};

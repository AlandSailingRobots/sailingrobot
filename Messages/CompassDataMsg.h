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

#include "Message.h"


class CompassDataMsg : public Message {
public:
	CompassDataMsg(NodeID destinationID, NodeID sourceID, int heading, int pitch, int roll)
		:Message(MessageType::CompassData, sourceID, destinationID), m_Heading(heading), m_Pitch(pitch), m_Roll(roll)
	{ }

	CompassDataMsg(int heading, int pitch, int roll)
		:Message(MessageType::CompassData, NodeID::None, NodeID::None), m_Heading(heading), m_Pitch(pitch), m_Roll(roll)
	{ }

	virtual ~CompassDataMsg() { }

	int heading() { return m_Heading; }
	int pitch() { return m_Pitch; }
	int roll() { return m_Roll; }

private:
	int m_Heading;
	int m_Pitch;
	int m_Roll;
};

/****************************************************************************************
 *
 * File:
 * 		CourseDataMsg.h
 *
 * Purpose:
 *		A CourseDataMsg contains information about the boats current course
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class CourseDataMsg : public Message {
public:
	CourseDataMsg(NodeID destinationID, NodeID sourceID, float twd, float distanceToWaypoint, float courseToWaypoint)
		:Message(MessageType::CourseData, sourceID, destinationID), m_twd(twd), m_dtw(distanceToWaypoint), m_ctw(courseToWaypoint) { }

	CourseDataMsg(float twd, float distanceToWaypoint, float courseToWaypoint)
		:Message(MessageType::CourseData, NodeID::None, NodeID::None), m_twd(twd), m_dtw(distanceToWaypoint), m_ctw(courseToWaypoint)
	{

	}

	CourseDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	deserialiser.readFloat(m_twd) ||
			deserialiser.readFloat(m_dtw) ||
			deserialiser.readFloat(m_ctw))
		{
			m_valid = false;
		}
	}

	virtual ~CourseDataMsg() { }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);
		serialiser.serialise(m_twd);
		serialiser.serialise(m_dtw);
		serialiser.serialise(m_ctw);
	}

	float trueWindDir() { return m_twd; }
	float distanceToWP() { return m_dtw; }
	float courseToWP() { return m_ctw; }

private:
	float m_twd; // True wind direction
	float m_dtw; // Distance to waypoint
	float m_ctw; // Course to waypoint
};

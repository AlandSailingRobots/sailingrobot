/****************************************************************************************
 *
 * File:
 * 		ObstaclePositionMsg.h
 *
 * Purpose:
 *		A message contain an obstacle's position.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class ObstaclePositionMsg : public Message {
public:
	ObstaclePositionMsg(NodeID destinationID, NodeID sourceID,
                        double longitude, double latitude, double radius)
		:Message(MessageType::ObstaclePosition, sourceID, destinationID),
         m_longitude(longitude), m_latitude(latitude), m_radius(radius)
	{ }

	ObstaclePositionMsg(double longitude, double latitude, double radius)
		:Message(MessageType::ObstaclePosition,
                 NodeID::None, NodeID::None),
         m_longitude(longitude), m_latitude(latitude), m_radius(radius)
	{ }

	ObstaclePositionMsg(MessageDeserialiser deserialiser)
			:Message(deserialiser)
	{
		if(	!deserialiser.readDouble(m_longitude) ||
			!deserialiser.readDouble(m_latitude)  ||
            !deserialiser.readDouble(m_radius))
		{
			m_valid = false;
		}
	}

	virtual ~ActuatorPositionMsg() { }

	double longitude() { return m_longitude; }
	double latitude() { return m_latitude; }
	double radius() { return m_radius; }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_longitude);
		serialiser.serialise(m_latitude);
        serialiser.serialise(m_radius);
	}

private:
	double m_longitude;
    double m_latitude;
    double m_radius;
};

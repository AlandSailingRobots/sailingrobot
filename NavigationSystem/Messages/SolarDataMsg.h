/****************************************************************************************
 *
 * File:
 * 		SolarDataMsg.h
 *
 * Purpose:
 *		An SolarDataMsg contains position, heading and time data for the solar tracker to align with the sun.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"


class SolarDataMsg : public Message {
public:
	SolarDataMsg(NodeID destinationID, NodeID sourceID, double latitude, double longitude, double heading, int hour, int min)
		:Message(MessageType::SolarData, sourceID, destinationID), m_latitude(latitude), m_longitude(longitude), m_heading(heading), m_Hour(hour), m_Min(min)
	{ }

	SolarDataMsg(double latitude, double longitude, double heading, int hour, int min)
		:Message(MessageType::SolarData, NodeID::None, NodeID::None), m_latitude(latitude), m_longitude(longitude), m_heading(heading), m_Hour(hour), m_Min(min)
	{ }

	SolarDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readDouble(m_latitude) ||
			!deserialiser.readDouble(m_longitude) ||
			!deserialiser.readDouble(m_heading) ||
			!deserialiser.readInt(m_Hour) ||
			!deserialiser.readInt(m_Min))
		{
			m_valid = false;
		}
	}

	virtual ~SolarDataMsg() { }

	double latitude() { return m_latitude; }
	double longitude() { return m_longitude; }
	double heading() { return m_heading; }
  int hour() { return m_Hour; }
	int min() { return m_Min; }
  ///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_latitude);
		serialiser.serialise(m_longitude);
		serialiser.serialise(m_heading);
		serialiser.serialise(m_Hour);
		serialiser.serialise(m_Min);
	}

private:
	double m_latitude;
	double m_longitude;
	double m_heading;
	int m_Hour;
	int m_Min;
 };

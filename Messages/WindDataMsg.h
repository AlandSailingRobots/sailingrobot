/****************************************************************************************
 *
 * File:
 * 		WindDataMsg.h
 *
 * Purpose:
 *		Contains the following data: Wind temperature, wind direction and wind speed.
 *		Commonly generated by a wind sensor
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class WindDataMsg : public Message {
public:
	WindDataMsg(NodeID destinationID, NodeID sourceID, float windDir, float windSpeed, float windTemp)
		:Message(MessageType::WindData, sourceID, destinationID),
		m_WindDir(windDir), m_WindSpeed(windSpeed), m_WindTemp(windTemp)
	{ }

	WindDataMsg(float windDir, float windSpeed, float windTemp)
		:Message(MessageType::WindData, NodeID::None, NodeID::None),
		m_WindDir(windDir), m_WindSpeed(windSpeed), m_WindTemp(windTemp)
	{ }

	WindDataMsg(MessageDeserialiser deserialiser)
		:Message(deserialiser)
	{
		if(	!deserialiser.readFloat(m_WindDir) ||
			!deserialiser.readFloat(m_WindSpeed) ||
			!deserialiser.readFloat(m_WindTemp))
		{
			m_valid = false;
		}
	}

	virtual ~WindDataMsg() { }

	float windDirection() const { return m_WindDir; }
	float windSpeed() const { return m_WindSpeed; }
	float windTemp() const { return m_WindTemp; }

	///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_WindDir);
		serialiser.serialise(m_WindSpeed);
		serialiser.serialise(m_WindTemp);
	}

private:
	float 	m_WindDir;
	float 	m_WindSpeed;
	float	m_WindTemp;
};

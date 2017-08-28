/****************************************************************************************
 *
 * File:
 * 		WaypointStationKeepingMsg.h
 *
 * Purpose:
 *		A WaypointStationKeepingMsg contains waypoint data such as id, longitude, latitude, declination and radius
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Message.h"


class WaypointStationKeepingMsg : public Message {
public:
	WaypointStationKeepingMsg(NodeID destinationID, NodeID sourceID, int id, double longitude, double latitude, int declination, int radius, 
                        int stayTime)
		:Message(MessageType::WaypointStationKeeping, sourceID, destinationID), m_id(id), m_longitude(longitude), m_latitude(latitude),
                         m_declination(declination), m_radius(radius), m_stayTime(stayTime)
	{ }

	WaypointStationKeepingMsg(int id, double longitude, double latitude, int declination, int radius, int stayTime)
		:Message(MessageType::WaypointStationKeeping, NodeID::None, NodeID::None), m_id(id), m_longitude(longitude), m_latitude(latitude),
                         m_declination(declination), m_radius(radius), m_stayTime(stayTime)

	{ }

	WaypointStationKeepingMsg(MessageDeserialiser deserialiser)
	:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_id) ||
			!deserialiser.readDouble(m_longitude) ||
			!deserialiser.readDouble(m_latitude) ||
			!deserialiser.readInt(m_declination) ||
			!deserialiser.readInt(m_radius) ||
			!deserialiser.readInt(m_stayTime))
		{
			m_valid = false;
		}
	}

	virtual ~WaypointStationKeepingMsg() { }


    int     id()            const { return m_id; }
    double  longitude()     const { return m_longitude; }
    double  latitude()      const { return m_latitude; }
    int     declination()   const { return m_declination; }
    int     radius()        const { return m_radius; }
    int     stayTime()          const { return m_stayTime; }

    ///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_id);
		serialiser.serialise(m_longitude);
		serialiser.serialise(m_latitude);
		serialiser.serialise(m_declination);
		serialiser.serialise(m_radius);
		serialiser.serialise(m_stayTime);
	}


private:

    int     m_id;
    double  m_longitude;
    double  m_latitude;
    int     m_declination;
    int     m_radius;
    int     m_stayTime;
};
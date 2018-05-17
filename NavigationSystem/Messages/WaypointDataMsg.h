/****************************************************************************************
 *
 * File:
 * 		WaypointDataMsg.h
 *
 * Purpose:
 *		A WaypointDataMsg contains waypoint data such as id, longitude, latitude, declination and radius
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "../MessageBus/Message.h"


class WaypointDataMsg : public Message {
public:
	WaypointDataMsg(NodeID destinationID, NodeID sourceID, int nextId, double nextLongitude, double nextLatitude, int nextDeclination, int nextRadius,
                        int nextStayTime, bool isCheckpoint, int prevId, double prevLongitude, double prevLatitude, int prevDeclination, int prevRadius)
		:Message(MessageType::WaypointData, sourceID, destinationID), m_nextId(nextId), m_nextLongitude(nextLongitude), m_nextLatitude(nextLatitude),
                         m_nextDeclination(nextDeclination), m_nextRadius(nextRadius), m_nextStayTime(nextStayTime),
						 m_isCheckpoint(isCheckpoint), m_prevId(prevId),
                         m_prevLongitude(prevLongitude), m_prevLatitude(prevLatitude), m_prevDeclination(prevDeclination), m_prevRadius(prevRadius)
	{ }

	WaypointDataMsg(int nextId, double nextLongitude, double nextLatitude, int nextDeclination, int nextRadius, int nextStayTime,
						bool isCheckpoint, int prevId, double prevLongitude, double prevLatitude, int prevDeclination, int prevRadius)
		:Message(MessageType::WaypointData, NodeID::None, NodeID::None), m_nextId(nextId), m_nextLongitude(nextLongitude), m_nextLatitude(nextLatitude),
                         m_nextDeclination(nextDeclination), m_nextRadius(nextRadius), m_nextStayTime(nextStayTime),
						 m_isCheckpoint(isCheckpoint), m_prevId(prevId), m_prevLongitude(prevLongitude),
                         m_prevLatitude(prevLatitude), m_prevDeclination(prevDeclination), m_prevRadius(prevRadius)

	{ }

	WaypointDataMsg(MessageDeserialiser deserialiser)
	:Message(deserialiser)
	{
		if(	!deserialiser.readInt(m_nextId) ||
			!deserialiser.readDouble(m_nextLongitude) ||
			!deserialiser.readDouble(m_nextLatitude) ||
			!deserialiser.readInt(m_nextDeclination) ||
			!deserialiser.readInt(m_nextRadius) ||
			!deserialiser.readInt(m_nextStayTime) ||
			!deserialiser.readBool(m_isCheckpoint) ||
			!deserialiser.readInt(m_prevId) ||
			!deserialiser.readDouble(m_prevLongitude) ||
			!deserialiser.readDouble(m_prevLatitude) ||
			!deserialiser.readInt(m_prevDeclination) ||
			!deserialiser.readInt(m_prevRadius))
		{
			m_valid = false;
		}
	}

	virtual ~WaypointDataMsg() { }


    int     nextId()            const { return m_nextId; }
    double  nextLongitude()     const { return m_nextLongitude; }
    double  nextLatitude()      const { return m_nextLatitude; }
    int     nextDeclination()   const { return m_nextDeclination; }
    int     nextRadius()        const { return m_nextRadius; }
    int     stayTime()          const { return m_nextStayTime; }
	bool	isCheckpoint()		const { return m_isCheckpoint; }

    int     prevId()            const { return m_prevId; }
    double  prevLongitude()     const { return m_prevLongitude; }
    double  prevLatitude()      const { return m_prevLatitude; }
    int     prevDeclination()   const { return m_prevDeclination; }
    int     prevRadius()        const { return m_prevRadius; }

    ///----------------------------------------------------------------------------------
	/// Serialises the message into a MessageSerialiser
	///----------------------------------------------------------------------------------
	virtual void Serialise(MessageSerialiser& serialiser) const
	{
		Message::Serialise(serialiser);

		serialiser.serialise(m_nextId);
		serialiser.serialise(m_nextLongitude);
		serialiser.serialise(m_nextLatitude);
		serialiser.serialise(m_nextDeclination);
		serialiser.serialise(m_nextRadius);
		serialiser.serialise(m_nextStayTime);
		serialiser.serialise(m_isCheckpoint);
		serialiser.serialise(m_prevId);
		serialiser.serialise(m_prevLongitude);
		serialiser.serialise(m_prevLatitude);
		serialiser.serialise(m_prevDeclination);
		serialiser.serialise(m_prevRadius);

		serialiser.serialise(m_isCheckpoint);
	}

private:

    int     m_nextId;
    double  m_nextLongitude;	// units : North(+) or South(-) [0-90]
    double  m_nextLatitude;		// units : East(+) or West(-)  [0-180]
    int     m_nextDeclination;	// units : degrees
    int     m_nextRadius;		// units : meters
    int     m_nextStayTime;		// units : seconds
	bool	m_isCheckpoint;		// units : true or false

    int     m_prevId;
    double  m_prevLongitude;	// units : North(+) or South(-) [0-90]
    double  m_prevLatitude;		// units : East(+) or West(-)  [0-180]
    int     m_prevDeclination;	// units : degrees
    int     m_prevRadius;		// units : meters
};

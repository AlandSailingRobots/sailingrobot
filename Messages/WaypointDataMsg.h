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

#include "Message.h"


class WaypointDataMsg : public Message {
public:
	WaypointDataMsg(NodeID destinationID, NodeID sourceID, int id, float longitude, float latitude, int declination, int radius)
		:Message(MessageType::WaypointData, sourceID, destinationID), m_id(id), m_longitude(longitude), m_latitude(latitude), m_declination(declination), m_radius(radius)
	{ }

	WaypointDataMsg(int id, float longitude, float latitude, int declination, int radius)
		:Message(MessageType::WaypointData, NodeID::None, NodeID::None), m_id(id), m_longitude(longitude), m_latitude(latitude), m_declination(declination), m_radius(radius)
	{ }

	virtual ~WaypointDataMsg() { }


    int id()            { return m_id; }
    float longitude()   { return m_longitude; }
    float latitude()    { return m_latitude; }
    int declination()   { return m_declination; }
    int radius()        { return m_radius; }

private:

    int     m_id;
    float   m_longitude;
    float   m_latitude;
    int     m_declination;
    int     m_radius;
};

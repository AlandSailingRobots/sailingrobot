/****************************************************************************************
 *
 * File:
 * 		WaypointNode.h
 *
 * Purpose:
 *		The WaypointNode sends information about the waypoints to the sailing logic
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"
#include "httpsync/HTTPSync.h"



class WaypointNode : public ActiveNode {
public:
	WaypointNode(MessageBus& msgBus);
    ~WaypointNode(){};

	bool init();


	void processMessage(const Message* msg);

    int id() { return m_id; }
    float longitude() { return m_longitude; }
    float latitude() { return m_latitude; }
    int declination() { return m_declination; }
    int radius() { return m_radius; }
    bool harvested() { return m_harvested; }

private:
    void getWaypointValues();

    int m_id;
    float m_longitude;
    float m_latitude;
    int m_declination;
    int m_radius;
    bool m_harvested;
};

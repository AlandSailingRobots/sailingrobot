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


#include "Node.h"
#include "httpsync/HTTPSync.h"
#include "Messages/GPSDataMsg.h"
#include "coursecalculation/CourseMath.h"



class WaypointNode : public Node {
public:
	WaypointNode(MessageBus& msgBus, DBHandler& db);
    virtual ~WaypointNode(){};

	bool init();

	void processMessage(const Message* message);

private:
	void processGPSMessage(GPSDataMsg* msg);
    bool waypointReached();

	///----------------------------------------------------------------------------------
 	/// Sends message with data about the next waypoint
 	///----------------------------------------------------------------------------------
    void sendMessage();

    DBHandler m_db;
    CourseMath m_courseMath;

    int m_id;
    float m_longitude;
    float m_latitude;
    int m_declination;
    int m_radius;

    float m_gps_longitude;
    float m_gps_latitude;
};

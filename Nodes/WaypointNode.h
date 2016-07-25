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

    int     m_nextId;
    double  m_nextLongitude;
    double  m_nextLatitude;
    int     m_nextDeclination;
    int     m_nextRadius;

    int     m_prevId;
    double  m_prevLongitude;
    double  m_prevLatitude;
    int     m_prevDeclination;
    int     m_prevRadius;

    double  m_gps_longitude;
    double  m_gps_latitude;
};

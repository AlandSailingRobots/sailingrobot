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


#include "MessageBus/Node.h"
#include "Messages/GPSDataMsg.h"
#include "DataBase/DBHandler.h"
#include "Math/CourseMath.h"
#include "SystemServices/Timer.h"


class WaypointMgrNode : public Node {
public:
	WaypointMgrNode(MessageBus& msgBus, DBHandler& db);
    virtual ~WaypointMgrNode(){};

	bool init();

	void processMessage(const Message* message);

private:
	void processGPSMessage(const GPSDataMsg* msg);
    bool waypointReached();

	///----------------------------------------------------------------------------------
 	/// Sends message with data about the next waypoint
 	///----------------------------------------------------------------------------------
    void sendMessage();
    bool harvestWaypoint();

    DBHandler &m_db;
    bool writeTime;

    int     m_nextId;
    double  m_nextLongitude;	// units : North(+) or South(-) [0-90]
    double  m_nextLatitude;		// units : East(+) or West(-)  [0-180]
    int     m_nextDeclination;	// NOTE : units ?
    int     m_nextRadius;		// NOTE : units ?
    int     m_nextStayTime;		// units : seconds

    int     m_prevId;
    double  m_prevLongitude;	// units : North(+) or South(-) [0-90]
    double  m_prevLatitude;		// units : East(+) or West(-)  [0-180]
    int     m_prevDeclination;	// NOTE : units ?
    int     m_prevRadius;		// NOTE : units ?

    double  m_gpsLongitude;		// units : North(+) or South(-) [0-90]
    double  m_gpsLatitude;		// units : East(+) or West(-)  [0-180]

    Timer   m_waypointTimer;	// units : seconds
    Timer   m_routeTime;		// units : seconds
    double  m_totalTime;		// units : seconds
};

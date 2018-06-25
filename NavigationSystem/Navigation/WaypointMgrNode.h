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

#include <string>
#include <vector>

#include "Database/DBHandler.h"
#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "MessageBus/Node.h"
#include "Messages/CourseDataMsg.h"
#include "Messages/ServerWaypointsReceivedMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/WaypointDataMsg.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

class WaypointMgrNode : public Node {
   public:
    WaypointMgrNode(MessageBus& msgBus, DBHandler& db);
    virtual ~WaypointMgrNode(){};

    bool init();

    void processMessage(const Message* message);

   private:
    void processVesselStateMessage(StateMessage* msg);
    bool waypointReached();

    ///----------------------------------------------------------------------------------
    /// Sends message with data about the next waypoint
    ///----------------------------------------------------------------------------------
    void sendMessage();
    bool harvestWaypoint();
    void sendNavigationInformation();

    DBHandler& m_db;
    bool writeTime;

    int m_nextId;
    double m_nextLongitude;  // units : North(+) or South(-) [0-90]
    double m_nextLatitude;   // units : East(+) or West(-)  [0-180]
    int m_nextDeclination;   // units : degrees
    int m_nextRadius;        // units : meters
    int m_nextStayTime;      // units : seconds

    bool m_isCheckpoint;  // units : true or false

    int m_prevId;
    double m_prevLongitude;  // units : North(+) or South(-) [0-90]
    double m_prevLatitude;   // units : East(+) or West(-)  [0-180]
    int m_prevDeclination;   // units : degrees
    int m_prevRadius;        // units : meters

    double m_vesselLongitude;  // units : North(+) or South(-) [0-90]
    double m_vesselLatitude;   // units : East(+) or West(-)  [0-180]

    Timer m_waypointTimer;  // units : seconds
    Timer m_routeTime;      // units : seconds
    double m_totalTime;     // units : seconds
};

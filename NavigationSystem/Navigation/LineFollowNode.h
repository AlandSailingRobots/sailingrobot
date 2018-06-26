/****************************************************************************************
 *
 * File:
 * 		LineFollowNode.h
 *
 * Purpose:
 *		This class computes the actuator positions of the boat in order to follow the
 *    line given by the waypoints.
 *
 * Developer Notes:
 *	 Algorithm inspired and modified from:
 *	 - Luc Jaulin and Fabrice Le Bars "An Experimental Validation of a Robust Controller with
 *		the VAIMOS Autonomous Sailboat" [1];
 *	 - Jon Melin, Kjell Dahl and Matia Waller "Modeling and Control for an Autonomous Sailboat:
 *   	A Case Study" [2].
 *
 *	 Info about Tacking and Beating : https://en.wikipedia.org/wiki/Tacking_(sailing)
 *
 ***************************************************************************************/

#pragma once

#include <math.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>

#include "../Database/DBHandler.h"
#include "../Math/CourseMath.h"
#include "../Math/Utility.h"
#include "../MessageBus/ActiveNode.h"
#include "../Messages/ExternalControlMsg.h"
#include "../Messages/LocalNavigationMsg.h"
#include "../Messages/StateMessage.h"
#include "../Messages/WaypointDataMsg.h"
#include "../Messages/WindStateMsg.h"
#include "../SystemServices/Timer.h"

class LineFollowNode : public ActiveNode {
   public:
    LineFollowNode(MessageBus& msgBus, DBHandler& dbhandler);
    ~LineFollowNode();

    bool init();
    void start();
    void stop();
    void processMessage(const Message* message);

   private:
    ///----------------------------------------------------------------------------------
    /// Updates the values of the parameters from the database.
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Stores vessel position datas from a StateMessage.
    ///----------------------------------------------------------------------------------
    void processStateMessage(const StateMessage* stateMsg);

    ///----------------------------------------------------------------------------------
    /// Stores Wind state datas from a WindStateMsg.
    ///----------------------------------------------------------------------------------
    void processWindStateMessage(const WindStateMsg* windStateMsg);

    ///----------------------------------------------------------------------------------
    /// Stores the waypoint positions and radius from a WaypointDataMsg.
    ///----------------------------------------------------------------------------------
    void processWaypointMessage(WaypointDataMsg* waypMsg);

    ///----------------------------------------------------------------------------------
    /// Calculates the angle of the line to be followed. in north east down reference frame.
    ///----------------------------------------------------------------------------------
    double calculateAngleOfDesiredTrajectory();

    ///----------------------------------------------------------------------------------
    /// Calculates the course to steer by using the line follow algorithm described in the papers.
    ///----------------------------------------------------------------------------------
    double calculateTargetCourse();

    ///----------------------------------------------------------------------------------
    /// Returns true if the desired tack of the vessel is starboard (wind blowing from the right
    /// side )
    ///----------------------------------------------------------------------------------
    bool getTargetTackStarboard(double targetCourse);

    ///----------------------------------------------------------------------------------
    /// If boat passed waypoint or enters it, set new line from boat to next waypoint.
    /// Used if boat has to stay within waypoint for a set amount of time (Station Keeping).
    ///----------------------------------------------------------------------------------
    void ifBoatPassedOrEnteredWP_setPrevWPToBoatPos();

    ///----------------------------------------------------------------------------------
    /// Starts the LineFollowNode's thread that pumps out LocalNavigationMsg.
    ///----------------------------------------------------------------------------------
    static void LineFollowNodeThreadFunc(ActiveNode* nodePtr);

    DBHandler& m_db;
    std::mutex m_lock;
    std::atomic<bool> m_Running;

    double m_LoopTime;  // seconds

    std::vector<float> m_TwdBuffer;  // True wind direction buffer. angles in degree [0, 360[ in
                                     // vessel reference frame (clockwise)

    // Vecteur field parameters
    float m_IncidenceAngle;       // radian.	[1]: (gamma_infiniy). 			[2]: (gamma).
    float m_MaxDistanceFromLine;  // meter.	[1]: cutoff distance (r). 		[2]: waypoint radius
                                  // (r).

    // Beating sailing mode parameters
    float m_CloseHauledAngle;  // radian. 	[1]: close hauled angle (zeta).	[2]: tacking angle
                               // (theta_t).
    float m_BroadReachAngle;   // radian.	Not in articles (new for downwind beating)
    float m_TackingDistance;   // meter. 	[1]: (r/2).						[2]: tacking distance
                               // (d).

    // Input variables
    bool m_externalControlActive;

    double m_VesselLat;
    double m_VesselLon;

    double m_trueWindSpeed;  // m/s
    double m_trueWindDir;    // degree [0, 360[ in North-East reference frame (clockwise)

    double m_nextWaypointLon;
    double m_nextWaypointLat;
    int m_nextWaypointRadius;  // m

    double m_prevWaypointLon;
    double m_prevWaypointLat;
    int m_prevWaypointRadius;  // m

    // State variable (inout variable)
    int m_TackDirection;  // [1] and [2]: tack variable (q).

    // Output variables
    bool m_BeatingMode;  // True if the vessel is in beating motion (zig-zag motion).
};

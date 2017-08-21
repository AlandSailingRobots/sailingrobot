/****************************************************************************************
*
* File:
*       StateEstimationNode.h
*
* Purpose:
*       Estimates the "current" state of the vessel. Collects datas from the GPS and compass messages.
*       Returns a VesselStateMsg corresponding at the estimated state of the vessel.
*
* Developer Notes:
*       Info about heading and magnetic direction : https://en.wikipedia.org/wiki/Course_(navigation)
*
*       Maël 26/07/17 : The magnetic variation used to correct the magnetic heading (which yields
*                       true heading) is the one at the next waypoint (setted up into the database)
*                       and not the magnetic variation at the current vessel position. So the correction
*                       won't be perfect when the vessel is far away from the next waypoint.
*
***************************************************************************************/

#pragma once

#include "MessageBus/ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

#include "DataBase/DBHandler.h"
#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include <mutex>
#include <stdint.h>
#include <atomic>


class StateEstimationNode : public ActiveNode {
public:
    StateEstimationNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime);
    StateEstimationNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime, double speed_1, double speed_2);
    ~StateEstimationNode();

    bool init();

    ///----------------------------------------------------------------------------------
    /// Starts the StateEstimationNode's thread that pumps out VesselStateMsg.
    ///----------------------------------------------------------------------------------
    void start();

    void stop();

    void processMessage(const Message* msg);



private:

    const int STATE_INITIAL_SLEEP = 2000;  //in milliseconds

    ///----------------------------------------------------------------------------------
    /// Stores compass data from a CompassDataMsg.
    ///----------------------------------------------------------------------------------
    void processCompassMessage(const CompassDataMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores the GPS data from a GPSDataMsg.
    ///----------------------------------------------------------------------------------
    void processGPSMessage(const GPSDataMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores the next declination from a WaypointDataMsg.
    ///----------------------------------------------------------------------------------
    void processWaypointMessage(const WaypointDataMsg* msg );

    ///----------------------------------------------------------------------------------
    /// Update values from the database as the loop time pf the thread
    /// and others parameters
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Estimates the vessel state from the sensor datas.
    ///----------------------------------------------------------------------------------
    bool estimateVesselState();

    ///----------------------------------------------------------------------------------
    /// Returns an estimation of the vessel course (angle of the velocity vector).
    /// When the vessel speed is sufficient (higher than m_speed_2), use the course over
    /// ground given by the GPS.
    /// When the vessel speed is not sufficient for the GPS to return good values (lower
    /// than m_speed_2) :
    ///     * speed_1 < VesselSpeed < speed_2 : use a combinaison of vesselHeading and GPSCourse
    ///     * VesselSpeed < speed_1 : use the vesselHeading
    ///----------------------------------------------------------------------------------
    float estimateVesselCourse();

    ///----------------------------------------------------------------------------------
    /// Starts the StateEstimationNode's thread that pumps out VesselStateMsg corresponding
    /// at the estimated state of the vessel.
    ///----------------------------------------------------------------------------------
    static void StateEstimationNodeThreadFunc(ActiveNode* nodePtr);


    double  m_LoopTime;             // second

    float   m_CompassHeading;       // degree [0, 360[ in North-East reference frame (clockwise)
    bool    m_GpsOnline;
    double  m_GPSLat;
    double  m_GPSLon;
    double  m_GPSSpeed;             // m/s
    double  m_GPSCourse;            // degree [0, 360[ in North-East reference frame (clockwise)
    float   m_WaypointDeclination;  // degree [??]

    double  m_speed_1;              // m/s
    double  m_speed_2;              // m/s

    float   m_VesselHeading;        // degree [0, 360[ in North-East reference frame (clockwise)
    double  m_VesselLat;
    double  m_VesselLon;
    float   m_VesselSpeed;          // m/s
    float   m_VesselCourse;         // degree [0, 360[ in North-East reference frame (clockwise)

    std::mutex        m_lock;
    std::atomic<bool> m_Running;
    DBHandler&        m_dbHandler;

};

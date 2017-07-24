/****************************************************************************************
*
* File:
* 		StateEstimationNode.h
*
*  Purpose:
*		Estimates the "current" state of the vessel. Collects datat from the GPS and compass messages.
*       Returns a VesselStateMsg corresponding at the estimated state of the vessel.
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include <stdint.h>
#include <mutex>

#include "MessageBus/ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WaypointDataMsg.h"


class StateEstimationNode : public ActiveNode {
public:
    StateEstimationNode(MessageBus& msgBus, double loopTime, double speedLimit);
    ~StateEstimationNode();

    bool init();

    ///----------------------------------------------------------------------------------
    /// Starts the StateEstimationNode's thread that pumps out VesselStateMsg.
    ///----------------------------------------------------------------------------------
    void start();

    void processMessage(const Message* msg);

private:

    const int STATE_INITIAL_SLEEP = 2000;

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
    /// Estimate the vessel course (angle of the velocity vector). When the vessel speed 
    /// is sufficient use the course over ground given by the GPS. When the vessel speed
    /// is not sufficient for the GPS to return good values use a combinaison of the vessel 
    /// heading and the GPS course.
    ///----------------------------------------------------------------------------------
    int estimateVesselCourse();

    ///----------------------------------------------------------------------------------
    /// Starts the StateEstimationNode's thread that pumps out StateMessages which contains
    /// data collected from the sensors
    ///----------------------------------------------------------------------------------
    static void StateEstimationNodeThreadFunc(ActiveNode* nodePtr);

    float m_VesselHeading;
    double m_VesselLat;
    double m_VesselLon;
    float m_VesselSpeed;
    float m_VesselCourse;

    double m_LoopTime;
    int m_WaypointDeclination;

    double m_SpeedLimit;
    bool m_GpsOnline;
    double  m_GPSLat;
    double  m_GPSLon;
    double  m_GPSSpeed;
    double  m_GPSCourse;

    std::mutex m_lock;


};

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

#include "Math/CourseMath.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"


class StateEstimationNode : public ActiveNode {
public:
    StateEstimationNode(MessageBus& msgBus, double loopTime);
    StateEstimationNode(MessageBus& msgBus, double loopTime, double speed_1, double speed_2);
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
    /// Starts the StateEstimationNode's thread that pumps out StateMessages which contains
    /// data collected from the sensors
    ///----------------------------------------------------------------------------------
    static void StateEstimationNodeThreadFunc(ActiveNode* nodePtr);


    double  m_LoopTime;

    float   m_CompassHeading;
    bool    m_GpsOnline;
    double  m_GPSLat;
    double  m_GPSLon;
    double  m_GPSSpeed;
    double  m_GPSCourse;
    int     m_WaypointDeclination;

    double  m_speed_1;
    double  m_speed_2;

    float   m_VesselHeading;
    double  m_VesselLat;
    double  m_VesselLon;
    float   m_VesselSpeed;
    float   m_VesselCourse;

    std::mutex m_lock;

};

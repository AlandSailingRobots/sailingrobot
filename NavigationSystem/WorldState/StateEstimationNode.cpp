/****************************************************************************************
*
* File:
* 		StateEstimationNode.cpp
*
* Purpose:
*       Estimates the "current" state of the vessel. Collects datat from the GPS and compass messages.
*       Returns a VesselStateMsg corresponding at the estimated state of the vessel.
*
* Developer Notes:
*
*
***************************************************************************************/

#include "StateEstimationNode.h"


StateEstimationNode::StateEstimationNode(MessageBus& msgBus, double loopTime, double speed_1, double speed_2): 
ActiveNode(NodeID::StateEstimation, msgBus), m_LoopTime(loopTime), m_CompassHeading(0), m_GpsOnline(false),
m_GPSLat(0), m_GPSLon(0), m_GPSSpeed(0), m_GPSCourse(0), m_WaypointDeclination(0), m_speed_1(speed_1), m_speed_2(speed_2),
m_VesselHeading(0), m_VesselLat(0), m_VesselLon(0), m_VesselSpeed(0), m_VesselCourse(0)
{
    msgBus.registerNode(*this, MessageType::CompassData);
    msgBus.registerNode(*this, MessageType::GPSData);
    msgBus.registerNode(*this, MessageType::WaypointData);
}

StateEstimationNode::StateEstimationNode(MessageBus& msgBus, double loopTime): 
ActiveNode(NodeID::StateEstimation, msgBus), m_LoopTime(loopTime), m_CompassHeading(0), m_GpsOnline(false),
m_GPSLat(0), m_GPSLon(0), m_GPSSpeed(0), m_GPSCourse(0), m_WaypointDeclination(0), m_speed_1(0), m_speed_2(1),
m_VesselHeading(0), m_VesselLat(0), m_VesselLon(0), m_VesselSpeed(0), m_VesselCourse(0)
{
    msgBus.registerNode(*this, MessageType::CompassData);
    msgBus.registerNode(*this, MessageType::GPSData);
    msgBus.registerNode(*this, MessageType::WaypointData);
}

StateEstimationNode::~StateEstimationNode() {}

bool StateEstimationNode::init()
{
    return true;
}

void StateEstimationNode::start()
{
    runThread(StateEstimationNodeThreadFunc);
}

void StateEstimationNode::processMessage(const Message* msg)
{
    MessageType type = msg->messageType();
    switch(type)
    {
    case MessageType::CompassData:
        processCompassMessage(static_cast<const CompassDataMsg*>(msg));
        break;
    case MessageType::GPSData:
        processGPSMessage(static_cast<const GPSDataMsg*> (msg));
        break;
    case MessageType::WaypointData:
        processWaypointMessage(static_cast<const WaypointDataMsg*> (msg));
        break;
    default:
        return;
    }
}

void StateEstimationNode::processCompassMessage(const CompassDataMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);
    m_CompassHeading = msg->heading();
}

void StateEstimationNode::processGPSMessage(const GPSDataMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);
    m_GpsOnline = msg->gpsOnline();
    m_GPSLat = msg->latitude();
    m_GPSLon = msg->longitude();
    m_GPSSpeed = msg->speed();
    m_GPSCourse = msg->course();
}

void StateEstimationNode::processWaypointMessage( const WaypointDataMsg* msg )
{
    std::lock_guard<std::mutex> lock_guard(m_lock);
    m_WaypointDeclination = msg->nextDeclination();
}

bool StateEstimationNode::estimateVesselState()
{   
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_VesselHeading = Utility::addDeclinationToHeading(m_CompassHeading, m_WaypointDeclination);
    if(m_GpsOnline)
    {
        m_VesselLat = m_GPSLat;
        m_VesselLon = m_GPSLon;
        m_VesselSpeed = m_GPSSpeed;
        m_VesselCourse = estimateVesselCourse();
        return true;
    }
    else
    {
        return false;
    }
}

float StateEstimationNode::estimateVesselCourse()
{
    if (m_speed_1 > m_speed_2) // Error. Need to be m_speed_1 <= m_speed_2.
    {
        m_speed_1 =  m_speed_2;
    }

    if(m_VesselSpeed < m_speed_1)
    {
        return m_VesselHeading;
    }
    else if(m_VesselSpeed >= m_speed_2)
    {
        return m_GPSCourse;
    }
    else // m_speed_1 <= m_VesselSpeed < m_speed_2
    {
        return Utility::linearFunctionBetweenAngle(m_VesselSpeed, m_speed_1, m_speed_2, m_VesselHeading, m_GPSCourse);
    }
}

void StateEstimationNode::StateEstimationNodeThreadFunc(ActiveNode* nodePtr)
{
    StateEstimationNode* node = dynamic_cast<StateEstimationNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the vessel state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(node->STATE_INITIAL_SLEEP));

    Timer timer;
    timer.start();

    while(true)
    {
        if(node->estimateVesselState())
        {
            MessagePtr stateMessage = std::make_unique<StateMessage>(node->m_VesselHeading, node->m_VesselLat,
                node->m_VesselLon, node->m_VesselSpeed, node->m_VesselCourse);
            node->m_MsgBus.sendMessage(std::move(stateMessage));
        }
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}

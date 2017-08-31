/****************************************************************************************
 *
 * File:
 * 		SpeedRegulatorNode.cpp
 *
 * Purpose:
 *      This file realize the regulation of the vessel's speed.
 *      It sends the value to the ???
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/
#include "SpeedRegulatorNode.h"
#include <chrono>
#include <thread>
#include <math.h>
//#include <cstdlib>
#include "Math/Utility.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

#define STATE_INITIAL_SLEEP 2000
#define HEADING_ERROR_VALUE 370

SpeedRegulatorNode::SpeedRegulatorNode( MessageBus& msgBus,  DBHandler& dbhandler, double loopTime):
ActiveNode(NodeID::SpeedRegulatorNode,msgBus), m_VesselHeading(HEADING_ERROR_VALUE), m_VesselSpeed(0),
m_DesiredHeading(HEADING_ERROR_VALUE),m_db(dbhandler), m_LoopTime(loopTime),pGain(1),iGain(1)
{
    msgBus.registerNode( *this, MessageType::StateMessage);
    msgBus.registerNode( *this, MessageType::DesiredCourse);
    //msgBus.registerNode( *this, MessageType::NavigationControl);
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived);
}

///----------------------------------------------------------------------------------
SpeedRegulatorNode::~SpeedRegulatorNode()
{

}

///----------------------------------------------------------------------------------
bool SpeedRegulatorNode::init(){ return true;}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::start()
{
    runThread(SpeedRegulatorNodeThreadFunc);
}

///----------------------------------------------------------------------------------
double SpeedRegulatorNode::getFrequencyThread()
{
    return m_LoopTime;
}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::processMessage( const Message* msg )
{
    MessageType type = msg->messageType();
    switch(type)
    {
        case MessageType::StateMessage:
        processStateMessage(static_cast< const StateMessage*>(msg));
        break;
        case MessageType::DesiredCourse:
        processDesiredCourseMessage(static_cast< const DesiredCourseMsg*>(msg)); //verify
        break;
        // case MessageType::NavigationControl:
        // processNavigationControlMessage(static_cast< const NavigationControlMsg*>(msg));
        // break;
        case MessageType::ServerConfigsReceived:
        updateConfigsFromDB();
        break;
        default:
        return;
        //Logger::info("Desired Course: %d Heading: %d", desiredHeading, heading);
    }
}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::processStateMessage(const StateMessage* msg)
{
    m_VesselHeading = msg->heading();
    m_VesselSpeed = msg->speed();
}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::processDesiredCourseMessage(const DesiredCourseMsg* msg)
{
    m_DesiredHeading = static_cast<double>(msg->desiredCourse());
}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::processNavigationControlMessage(const NavigationControlMsg* msg)
{
    //std::lock_guard<std::mutex> lock_guard(m_lock);
    //m_NavigationState = msg->navigationState();
    // Not use
    //m_CourseToSteer = msg->courseToSteer();
    //m_TargetSpeed = msg->targetSpeed();
    //Windvane?
    //m_Tack = msg->tack(); // useful ?
    // No definition for the starboard
}

///----------------------------------------------------------------------------------
double SpeedRegulatorNode::calculateRudderAngle()
{
    if((m_DesiredHeading != HEADING_ERROR_VALUE) and (m_VesselHeading != HEADING_ERROR_VALUE)){

        double difference_Heading = Utility::limitAngleRange(m_VesselHeading) - Utility::limitAngleRange(m_DesiredHeading);
        // Equation from book "Robotic Sailing 2015 ", page 141
        // The MAX_RUDDER_ANGLE is a parameter configuring the variation around the desired heading.
        // Also the reaction could be configure by the frequence of the thread.
        if(cos(Utility::degreeToRadian(difference_Heading)) < 0) // Wrong sense because over 90°
        {
            // Max Rudder angle in the opposite way
            return Utility::sgn(m_VesselSpeed)*(Utility::sgn(sin(Utility::degreeToRadian(difference_Heading))))*m_MaxRudderAngle;
        }
        // Regulation of the rudder
        return Utility::sgn(m_VesselSpeed)*(sin(Utility::degreeToRadian(difference_Heading))*m_MaxRudderAngle);

    }
    return 0;
}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::updateFrequencyThread()
{
    m_LoopTime = m_db.retrieveCellAsDouble("sailing_robot_config","1","loop_time");
}

///----------------------------------------------------------------------------------
void SpeedRegulatorNode::SpeedRegulatorNodeThreadFunc(ActiveNode* nodePtr)
{
    SpeedRegulatorNode* node = dynamic_cast<SpeedRegulatorNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(STATE_INITIAL_SLEEP));

    Timer timer;
    timer.start();

    while(true)
    {

        node->m_lock.lock();
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(node->calculateRudderAngle(),0);
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));
        node->m_lock.unlock();
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
        node->updateFrequencyThread();
    }
}

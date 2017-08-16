/****************************************************************************************
 *
 * File:
 * 		CourseRegulatorNode.cpp
 *
 * Purpose:
 *      This file realize the regulation of the rudder by the desired course.
 *      It sends the value to the rudder actuator Node
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/
#include "CourseRegulatorNode.h"
#include <chrono>
#include <thread>
#include <math.h>
//#include <cstdlib>
#include "Math/Utility.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

const int ERROR_VALUE = 370;
const int STATE_INITIAL_SLEEP = 2000;
const float NO_COMMAND = -2000; 


CourseRegulatorNode::CourseRegulatorNode( MessageBus& msgBus, DBHandler& dbhandler, double loopTime, double maxRudderAngle,
    double configPGain, double configIGain):ActiveNode(NodeID::CourseRegulatorNode,msgBus), m_VesselCourse(ERROR_VALUE), m_VesselSpeed(0),
    m_MaxRudderAngle(maxRudderAngle),m_DesiredCourse(ERROR_VALUE),m_db(dbhandler), m_LoopTime(loopTime),pGain(configPGain),iGain(configIGain)
{
    msgBus.registerNode( *this, MessageType::StateMessage);
    msgBus.registerNode( *this, MessageType::DesiredCourse);
    msgBus.registerNode( *this, MessageType::LocalNavigation);
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived);
}

///----------------------------------------------------------------------------------
CourseRegulatorNode::~CourseRegulatorNode(){}

///----------------------------------------------------------------------------------
bool CourseRegulatorNode::init()
{ 
    updateConfigsFromDB();
    return true;
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::start()
{
    runThread(CourseRegulatorNodeThreadFunc);
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::updateConfigsFromDB()
{
    m_LoopTime = 0.5; //m_db.retrieveCellAsDouble("sailing_robot_config","1","loop_time");
    m_MaxRudderAngle = 30;
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processMessage( const Message* msg )
{
    switch(msg->messageType())
    {
    case MessageType::StateMessage:
        processStateMessage(static_cast< const StateMessage*>(msg));
        break;
    case MessageType::DesiredCourse:
        processDesiredCourseMessage(static_cast< const DesiredCourseMsg*>(msg)); //verify
        break;
    case MessageType::LocalNavigation:
        processLocalNavigationMessage(static_cast< const LocalNavigationMsg*>(msg));
        break;
    case MessageType::ServerConfigsReceived:
        updateConfigsFromDB();
        break;
    default:
        return;
    }
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processStateMessage(const StateMessage* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_VesselCourse = msg->course();
    m_VesselSpeed = msg->speed();
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processDesiredCourseMessage(const DesiredCourseMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_DesiredCourse = static_cast<double>(msg->desiredCourse());
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processLocalNavigationMessage(const LocalNavigationMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_DesiredCourse = msg->targetCourse();
}

///----------------------------------------------------------------------------------
float CourseRegulatorNode::calculateRudderAngle()
{
    if((m_DesiredCourse != ERROR_VALUE) and (m_VesselCourse != ERROR_VALUE))
    {
        double difference_Heading = Utility::limitAngleRange(m_VesselCourse) - Utility::limitAngleRange(m_DesiredCourse);
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
    else
    {
        return NO_COMMAND;
    }
}
    

///----------------------------------------------------------------------------------
void CourseRegulatorNode::CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr)
{
    CourseRegulatorNode* node = dynamic_cast<CourseRegulatorNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(STATE_INITIAL_SLEEP));

    Timer timer;
    timer.start();

    while(true)
    {

        node->m_lock.lock();
        //std::lock_guard<std::mutex> lock_guard(node->m_lock);
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(node->calculateRudderAngle(),0);
        //std::cout << std::endl << "COURSE REG NODE ######### Send : CalcR " << node->calculateRudderAngle();
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));
        node->m_lock.unlock();

        // Broadcast() or selected sent???
        timer.sleepUntil(node->m_LoopTime); //insert updateFrequencyThread in the function ?
        timer.reset();
    }
}

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
#include "Messages/RudderCommandMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

const int STATE_INITIAL_SLEEP = 2000;
#define DATA_OUT_OF_RANGE -2000

CourseRegulatorNode::CourseRegulatorNode( MessageBus& msgBus,  DBHandler& dbhandler)
:ActiveNode(NodeID::CourseRegulatorNode,msgBus), m_VesselCourse(DATA_OUT_OF_RANGE), m_VesselSpeed(0),
m_MaxRudderAngle(30),m_DesiredCourse(DATA_OUT_OF_RANGE),m_db(dbhandler), m_LoopTime(0.5),
pGain(1),iGain(1),dGain(1)
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
    m_Running.store(true);
    runThread(CourseRegulatorNodeThreadFunc);
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::stop()
{
    m_Running.store(false);
    stopThread(this);
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::updateConfigsFromDB()
{
    m_LoopTime = m_db.retrieveCellAsDouble("config_course_regulator","1","loop_time");
    m_MaxRudderAngle = m_db.retrieveCellAsInt("config_course_regulator","1","max_rudder_angle");
    pGain = m_db.retrieveCellAsDouble("config_course_regulator","1","p_gain");
    iGain = m_db.retrieveCellAsDouble("config_course_regulator","1","i_gain");
    dGain = m_db.retrieveCellAsDouble("config_course_regulator","1","d_gain");
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

    m_DesiredCourse = static_cast<float>(msg->desiredCourse());
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
    std::lock_guard<std::mutex> lock_guard(m_lock);

    if((m_DesiredCourse != DATA_OUT_OF_RANGE) and (m_VesselCourse != DATA_OUT_OF_RANGE))
    {
        // Equation from book "Robotic Sailing 2015 ", page 141
        // The m_MaxRudderAngle is a parameter configuring the variation around the desired heading.
        // Also the reaction could be configure by the frequence of the thread.

        //std::cout << "m_DesiredCourse : " << m_DesiredCourse <<std::endl;
        //std::cout << "m_VesselCourse : " << m_VesselCourse <<std::endl;

        float difference_Heading = Utility::degreeToRadian(m_VesselCourse - m_DesiredCourse);

        if(cos(difference_Heading) < 0) // Wrong sense because over +/- 90°
        {
            // Max Rudder angle in the opposite way
            return Utility::sgn(sin(difference_Heading))*m_MaxRudderAngle;
        }
        else
        {   // Regulation of the rudder
            return sin(difference_Heading)*m_MaxRudderAngle;
        }
    }
    else
    {
        return DATA_OUT_OF_RANGE;
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

    while(node->m_Running.load() == true)
    {
        float rudderCommand = node->calculateRudderAngle();
        if (rudderCommand != DATA_OUT_OF_RANGE)
        {

            // std::cout << "rudder command : " << rudderCommand <<std::endl;
            MessagePtr actuatorMessage = std::make_unique<RudderCommandMsg>(rudderCommand);
            node->m_MsgBus.sendMessage(std::move(actuatorMessage));
        }
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}

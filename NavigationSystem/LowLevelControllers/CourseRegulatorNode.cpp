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


#define DATA_OUT_OF_RANGE -2000
const int INITIAL_SLEEP = 2000; // milliseconds
const float NO_COMMAND = -1000;


///----------------------------------------------------------------------------------
CourseRegulatorNode::CourseRegulatorNode( MessageBus& msgBus,  DBHandler& dbhandler)
:ActiveNode(NodeID::CourseRegulatorNode,msgBus), m_db(dbhandler), m_LoopTime(0.5), m_MaxRudderAngle(30),
m_pGain(1), m_iGain(1), m_dGain(1),
m_VesselCourse(DATA_OUT_OF_RANGE), m_VesselSpeed(DATA_OUT_OF_RANGE), m_DesiredCourse(DATA_OUT_OF_RANGE)

{
    msgBus.registerNode( *this, MessageType::StateMessage);
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
    m_pGain = m_db.retrieveCellAsDouble("config_course_regulator","1","p_gain");
    m_iGain = m_db.retrieveCellAsDouble("config_course_regulator","1","i_gain");
    m_dGain = m_db.retrieveCellAsDouble("config_course_regulator","1","d_gain");
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processMessage( const Message* msg )
{
    switch(msg->messageType())
    {
    case MessageType::StateMessage:
        processStateMessage(static_cast< const StateMessage*>(msg));
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
        float difference_Heading = Utility::degreeToRadian(m_VesselCourse - m_DesiredCourse);

        if(cos(difference_Heading) < 0) // Wrong sense because over +/- 90°
        {   // Max Rudder angle in the opposite way
            return Utility::sgn(sin(difference_Heading))*m_MaxRudderAngle;
        }
        else
        {   // Regulation of the rudder
            return sin(difference_Heading)*m_MaxRudderAngle;
        }
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
    std::this_thread::sleep_for(std::chrono::milliseconds(INITIAL_SLEEP));

    Timer timer;
    timer.start();

    while(node->m_Running.load() == true)
    {
        float rudderCommand = node->calculateRudderAngle();
        if (rudderCommand != NO_COMMAND)
        {
            MessagePtr actuatorMessage = std::make_unique<RudderCommandMsg>(rudderCommand);
            node->m_MsgBus.sendMessage(std::move(actuatorMessage));
        }
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}

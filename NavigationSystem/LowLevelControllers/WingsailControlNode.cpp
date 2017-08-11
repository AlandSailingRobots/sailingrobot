/****************************************************************************************
 *
 * File:
 * 		WingsailControlNode.cpp
 *
 * Purpose:
 *      This file realize the control of the wingsail by the wind state.
 *      It sends the direct command to the actuator who control the sail queue angle.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#include "WingsailControlNode.h"
#include <thread>
#include <math.h>
#include <mutex>
#include <chrono>
//#include "All.h" // library for M_PI
#include "Messages/WindDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"
#include "Math/Utility.h"

#define STATE_INITIAL_SLEEP 2000

WingsailControlNode::WingsailControlNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime)
:ActiveNode(NodeID::WingsailControlNode,msgBus),m_MaxWingsailAngle(43),m_MinWingsailAngle(5.5)
,m_ApparentWindDir(0),m_db(dbhandler),m_LoopTime(loopTime)
{
    msgBus.registerNode( *this, MessageType::WindData);
    msgBus.registerNode( *this, MessageType::NavigationControl);
}

///----------------------------------------------------------------------------------
WingsailControlNode::~WingsailControlNode(){}

///----------------------------------------------------------------------------------
bool WingsailControlNode::init(){ return true;}

///----------------------------------------------------------------------------------
void WingsailControlNode::start()
{
    runThread(WingsailControlNodeThreadFunc);
}

///----------------------------------------------------------------------------------
void WingsailControlNode::processMessage( const Message* msg)
{
    switch( msg->messageType() )
    {
        case MessageType::WindData:
        processWindDataMessage(static_cast< const WindDataMsg*>(msg));
        break;
        case MessageType::NavigationControl:
        processNavigationControlMessage(static_cast< const NavigationControlMsg*>(msg));
        break;
        default:
        return;
    }
}

///----------------------------------------------------------------------------------
void WingsailControlNode::processWindDataMessage(const WindDataMsg* msg)
{
    m_ApparentWindDir = msg->windDirection();
}

///----------------------------------------------------------------------------------
void WingsailControlNode::processNavigationControlMessage(const NavigationControlMsg* msg)
{
    //m_NavigationState = msg->navigationState();
}

//*----------------------------------------------------------------------------------
double WingsailControlNode::calculateWingsailAngle()
{
    // Equation from book "Robotic Sailing 2015 ", page 141
    // Also the reaction could be configure by the frequence of the thread.
    return -Utility::sgn(m_ApparentWindDir)*(((m_MinWingsailAngle-m_MaxWingsailAngle)*std::abs(m_ApparentWindDir)/180)+m_MaxWingsailAngle);
}
//*///----------------------------------------------------------------------------------

/*----------------------------------------------------------------------------------
double WingsailControlNode::calculateSailAngle()
{
    // Equation from book "Robotic Sailing 2012 ", page 109
    // Also the reaction could be configure by the frequence of the thread.
    return (m_MaxWingsailAngle-m_MinWingsailAngle)*((cos(Utility::degreeToRadian(m_ApparentWindDir))+1)/2 + 1);
}
//*///----------------------------------------------------------------------------------

///----------------------------------------------------------------------------------
double WingsailControlNode::getFrequencyThread()
{
    return m_LoopTime;
}

///----------------------------------------------------------------------------------
void WingsailControlNode::updateFrequencyThread()
{
    m_LoopTime = m_db.retrieveCellAsDouble("config_wingsail_control","1","loop_time"); //see next table
    m_MaxSailAngle = m_db.retrieveCellAsDouble("config_wingsail_control","1","loop_time"); //see next table
    m_MinSailAngle = m_db.retrieveCellAsDouble("config_wingsail_control","1","loop_time"); //see next table

}

///----------------------------------------------------------------------------------
void WingsailControlNode::WingsailControlNodeThreadFunc(ActiveNode* nodePtr)
{
    WingsailControlNode* node = dynamic_cast<WingsailControlNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(STATE_INITIAL_SLEEP));
    Timer timer;
    timer.start();

    while(true)
    {
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(0,node->calculateWingsailAngle());
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
        node->updateFrequencyThread();
    }
}

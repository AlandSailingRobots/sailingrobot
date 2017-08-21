/****************************************************************************************
 *
 * File:
 * 		SailControlNode.cpp
 *
 * Purpose:
 *      This file realize the control of the sail by the wind state.
 *      It sends the direct command to the actuator who control the sail sheet.
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#include "SailControlNode.h"
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

SailControlNode::SailControlNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime)
    :ActiveNode(NodeID::SailControlNode,msgBus),m_MaxSailAngle(90),m_MinSailAngle(10),
    m_ApparentWindDir(0),m_db(dbhandler),m_LoopTime(loopTime)
{
    msgBus.registerNode( *this, MessageType::WindData);
    msgBus.registerNode( *this, MessageType::LocalNavigation);
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived);
}

///----------------------------------------------------------------------------------
SailControlNode::~SailControlNode()
{

}

///----------------------------------------------------------------------------------
bool SailControlNode::init(){ return true;}

///----------------------------------------------------------------------------------
void SailControlNode::start()
{
    m_Running.store(true);
    runThread(SailControlNodeThreadFunc);
}

///----------------------------------------------------------------------------------
void SailControlNode::stop()
{
    m_Running.store(false);
    stopThread(this);
}

///----------------------------------------------------------------------------------
void SailControlNode::updateConfigsFromDB()
{
    m_LoopTime = m_db.retrieveCellAsDouble("config_sail_control","1","loop_time");
    m_MaxSailAngle = m_db.retrieveCellAsInt("config_sail_control","1","max_sail_angle");
    m_MinSailAngle = m_db.retrieveCellAsInt("config_sail_control","1","min_sail_angle");
}

///----------------------------------------------------------------------------------
void SailControlNode::processMessage( const Message* msg)
{
    switch( msg->messageType() )
    {
        case MessageType::WindData:
        processWindDataMessage(static_cast< const WindDataMsg*>(msg));
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
void SailControlNode::processWindDataMessage(const WindDataMsg* msg)
{
    m_ApparentWindDir = msg->windDirection();
}

///----------------------------------------------------------------------------------
void SailControlNode::processLocalNavigationMessage(const LocalNavigationMsg* msg)
{
    //std::lock_guard<std_mutex> lock_guard(m_lock);
    //m_NavigationState = msg->navigationState();
}

//*----------------------------------------------------------------------------------
double SailControlNode::calculateSailAngle()
{
    // Equation from book "Robotic Sailing 2015 ", page 141
    // Also the reaction could be configure by the frequence of the thread.
    return -Utility::sgn(m_ApparentWindDir)*(((m_MinSailAngle-m_MaxSailAngle)*std::abs(m_ApparentWindDir)/180)+m_MaxSailAngle);
}
//*///----------------------------------------------------------------------------------

/*----------------------------------------------------------------------------------
double SailControlNode::calculateSailAngle()
{
    // Equation from book "Robotic Sailing 2012 ", page 109
    // Also the reaction could be configure by the frequence of the thread.
    return (m_MaxSailAngle-m_MinSailAngle)*((cos(Utility::degreeToRadian(m_ApparentWindDir))+1)/2 + 1);
}
//*///----------------------------------------------------------------------------------

///----------------------------------------------------------------------------------
void SailControlNode::SailControlNodeThreadFunc(ActiveNode* nodePtr)
{
    SailControlNode* node = dynamic_cast<SailControlNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(STATE_INITIAL_SLEEP));
    Timer timer;
    timer.start();

    while(node->m_Running.load() == true)
    {
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(0,node->calculateSailAngle());
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}

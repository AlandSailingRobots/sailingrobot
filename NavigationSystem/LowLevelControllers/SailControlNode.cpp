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


SailControlNode::SailControlNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime, double maxSailAngle,
    double minSailAngle, double maxCommandAngle, double configPGain, double configIGain):ActiveNode(NodeID::SailControlNode,msgBus),
    m_MaxSailAngle(maxSailAngle),m_MinSailAngle(minSailAngle),m_MaxCommandAngle(maxCommandAngle),m_ApparentWindDir(0),pGain(configPGain),
    iGain(configIGain),m_db(dbhandler),m_LoopTime(loopTime)
{
    msgBus.registerNode( *this, MessageType::WindData);
    msgBus.registerNode( *this, MessageType::NavigationControl);
}

///----------------------------------------------------------------------------------
SailControlNode::~SailControlNode(){}

///----------------------------------------------------------------------------------
bool SailControlNode::init(){ return true;}

///----------------------------------------------------------------------------------
void SailControlNode::start()
{
    runThread(SailControlNodeThreadFunc);
}

///----------------------------------------------------------------------------------
void SailControlNode::processMessage( const Message* msg)
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
void SailControlNode::processWindDataMessage(const WindDataMsg* msg)
{
    //std::lock_guard<std_mutex> lock_guard(m_lock);
    m_ApparentWindDir = msg->windDirection();
}

///----------------------------------------------------------------------------------
void SailControlNode::processNavigationControlMessage(const NavigationControlMsg* msg)
{
    //std::lock_guard<std_mutex> lock_guard(m_lock);
    //m_NavigationState = msg->navigationState();
}

///----------------------------------------------------------------------------------
double SailControlNode::restrictSail(double val)
{
    if( val > m_MaxCommandAngle)        { return m_MaxCommandAngle; }
    else if ( val < -m_MaxCommandAngle) { return -m_MaxCommandAngle; }
    return val;
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
double SailControlNode::getFrequencyThread()
{
    return m_LoopTime;
}

///----------------------------------------------------------------------------------
void SailControlNode::updateFrequencyThread()
{
    m_LoopTime = m_db.retrieveCellAsDouble("___","1","loop_time"); //see next table
}

///----------------------------------------------------------------------------------
void SailControlNode::SailControlNodeThreadFunc(ActiveNode* nodePtr)
{
    SailControlNode* node = dynamic_cast<SailControlNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    Timer timer;
    timer.start();

    while(true)
    {
        std::lock_guard<std::mutex> lock_guard(node->m_lock);
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(0,node->calculateSailAngle());
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));

        // Broadcast() or selected sent???
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
        node->updateFrequencyThread();
    }
}

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
#include <math>
#include <mutex>
#include <chrono>
#include "All.h" // library for M_PI
#include "Messages/WindStateMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

SailControlNode::SailControlNode(MessageBus& msgBus, double loopTime, float maxSailAngle, float minSailAngle,
    float maxCommandAngle, double configPGain, double configIGain, DBHandler dbhandler)
    :ActiveNode(NodeID::SailControlNode,msgBus),m_MaxSailAngle(maxSailAngle),m_MinSailAngle(minSailAngle),
    m_MaxCommandAngle(maxCommandAngle),m_ApparentWindDir(0),pGain(configPGain),iGain(configIGain),m_db(dbhandler),m_LoopTime(loopTime)
{
    msgBus.registerNode( *this, MessageType::WindState);
    msgBus.registerNode( *this, MessageType::NavigationControl);
}

///----------------------------------------------------------------------------------
SailControlNode::~SailControlNode(){}

///----------------------------------------------------------------------------------
bool init(){ return true;}

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
        case MessageType::WindState:
        processWindStateMessage(static_cast< const WindStateMsg*>(msg));
        break;
        case MessageType::NavigationControlMsg:
        processNavigationControlMessage(static_cast< const NavigationControlMsg*>(msg));
        break;
        default:
        return;
    }
}

///----------------------------------------------------------------------------------
void SailControlNode::processWindStateMessage(const WindStateMsg* msg)
{
    std::lock_guard<std_mutex> lock_guard(m_lock);
    m_ApparentWindDir = msg->apparentWindDirection();
}

///----------------------------------------------------------------------------------
void SailControlNode::processNavigationControlMessage(const NavigationControlMsg* msg)
{
    std::lock_guard<std_mutex> lock_guard(m_lock);
    m_NavigationState = msg->navigationState();
}

///----------------------------------------------------------------------------------
double SailControlNode::restrictSail(double val)
{
    if( val > m_MaxCommandAngle)        { return m_MaxCommandAngle; }
    else if ( val < -m_MaxCommandAngle) { return -m_MaxCommandAngle; }
    return val;
}

///----------------------------------------------------------------------------------
double SailControlNode::calculateSailAngle()
{
    // Equation from book "Robotic Sailing 2015 ", page 141
    // Also the reaction could be configure by the frequence of the thread.
    return -sgn(m_ApparentWindDir)*(((m_MinSailAngle-m_MaxSailAngle)*std::abs(m_ApparentWindDir)/M_PI)+m_MaxSailAngle);
}

/*----------------------------------------------------------------------------------
double SailControlNode::calculateSailAngle()
{
    // Equation from book "Robotic Sailing 2012 ", page 109
    // Also the reaction could be configure by the frequence of the thread.
    return -sgn(m_ApparentWind)*(((m_MinSailAngle-m_MaxSailAngle)*std::abs(m_ApparentWind)/M_PI)+m_MaxSailAngle);
}
----------------------------------------------------------------------------------*/


///----------------------------------------------------------------------------------
void SailControlNode::updateFrequencyThread(SailControlNode* node)
{
    node->m_LoopTime = m_db.retrieveCellAsInt("sail_servo_config","1","loopTime");
}

///----------------------------------------------------------------------------------
void SailControlNode::SailControlNodeThreadFunc(ActiveNode* nodePtr)
{
    SailControlNode* node = dynamic_cast<SailControlNode> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(node->STATE_INITIAL_SLEEP))

    Timer timer;
    timer.start();

    while(true)
    {
        std::lock_guard<std::mutex> lock_guard(node->m_lock);
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMessage>(node->calculateSailAngle());
        node->m_MsgBus.sendMessage(std::move(actuatorMessage))
    
        // Broadcast() or selected sent???
        timer.sleepUntil(node->m_LoopTime)
        timer.reset();
        node->updateFrequencyThread(node); 
    }
}
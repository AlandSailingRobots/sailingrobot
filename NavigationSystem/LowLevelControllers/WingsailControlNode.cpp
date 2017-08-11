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
#include <list.h>
//#include "All.h" // library for M_PI
#include "Messages/WindDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"
#include "Math/Utility.h"


#DEFINE LIFTS = [];
#DEFINE DRAGS = [];


WingsailControlNode::WingsailControlNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime, double maxSailAngle,
    double minSailAngle, double maxCommandAngle, double configPGain, double configIGain):ActiveNode(NodeID::WingsailControlNode,msgBus),
    m_MaxWingsailAngle(maxSailAngle),m_MinWingsailAngle(minSailAngle),m_MaxCommandAngle(maxCommandAngle),m_ApparentWindDir(0),pGain(configPGain),
    iGain(configIGain),m_db(dbhandler),m_LoopTime(loopTime)
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
    //std::lock_guard<std_mutex> lock_guard(m_lock);
    m_ApparentWindDir = msg->windDirection();
}

///----------------------------------------------------------------------------------
void WingsailControlNode::processNavigationControlMessage(const NavigationControlMsg* msg)
{
    //std::lock_guard<std_mutex> lock_guard(m_lock);
    //m_NavigationState = msg->navigationState();
}

///----------------------------------------------------------------------------------
double WingsailControlNode::restrictWingsail(double val)
{
    if( val > m_MaxCommandAngle)        { return m_MaxCommandAngle; }
    else if ( val < -m_MaxCommandAngle) { return -m_MaxCommandAngle; }
    return val;
}

//*----------------------------------------------------------------------------------
double WingsailControlNode::calculateWingsailAngle()
{
    // Equation from book "Robotic Sailing 2015 ", page 141
    // Also the reaction could be configure by the frequence of the thread.
    return -Utility::sgn(m_ApparentWindDir)*(((m_MinWingsailAngle-m_MaxWingsailAngle)*std::abs(m_ApparentWindDir)/180)+m_MaxWingsailAngle);
}
//*///----------------------------------------------------------------------------------

///------------------------------------------------------------------------------------
/* Return the angle to give to the tail to have maximum force toward
 * boat heading */

double WingsailControlNode::calculateTailAngle()
{
    // lists that will contain the forces on X and Y in the boat coordinates system
    float xBoat_Forces [53];  
    float yBoat_Forces [53];
    
    int i;
    // transforming given calculated lifts and drags into forces in
    // the boat coordinates system
    for (i = 0;i < 54;i++)
    {
	*(xBoat_Forces+i) = cos(*(DRAGS + i)) - sin (*(LIFTS + i));
	*(yBoat_Forces+i) = cos(*(LIFTS + i)) + sin (*(DRAGS + i));
    }

    list::list maxAndIndex_xBoat_Forces;
    maxAndIndex_xBoat_Forces = Utility::maxAndIndex(xBoat_Forces);

    double orderTail;
    orderTail = maxAndIndex_xBoat_Forces.back() - 26;

    return(orderTail)
}



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
    m_LoopTime = m_db.retrieveCellAsDouble("___","1","loop_time"); //see next table
}

///----------------------------------------------------------------------------------
void WingsailControlNode::WingsailControlNodeThreadFunc(ActiveNode* nodePtr)
{
    WingsailControlNode* node = dynamic_cast<WingsailControlNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    Timer timer;
    timer.start();

    while(true)
    {
        std::lock_guard<std::mutex> lock_guard(node->m_lock);
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(0,node->calculateWingsailAngle());
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));

        // Broadcast() or selected sent???
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
        node->updateFrequencyThread();
    }
}

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
#include <vector>
//#include "All.h" // library for M_PI
#include "Messages/WindDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"
#include "Math/Utility.h"


#DEFINE LIFTS  [15.964797400676879, 15.350766731420075, 14.736736062163272, 14.122705392906468, 13.508674723649664, 12.894644054392863, 12.280613385136059, 11.666582715879256, 11.052552046622454, 10.438521377365651, 9.8244907081088488, 9.2104600388520446, 8.5964293695952421, 7.9823987003384396, 7.3683680310816362, 6.7543373618248319, 6.1403066925680294, 5.5262760233112269, 4.9122453540544244, 4.298214684797621, 3.6841840155408181, 3.0701533462840147, 2.4561226770272122, 1.842092007770409, 1.2280613385136061, 0.61403066925680305, 0.0, -0.61403066925680305, -1.2280613385136061, -1.842092007770409, -2.4561226770272122, -3.0701533462840147, -3.6841840155408181, -4.298214684797621, -4.9122453540544244, -5.5262760233112269, -6.1403066925680294, -6.7543373618248319, -7.3683680310816362, -7.9823987003384396, -8.5964293695952421, -9.2104600388520446, -9.8244907081088488, -10.438521377365651, -11.052552046622454, -11.666582715879256, -12.280613385136059, -12.894644054392863, -13.508674723649664, -14.122705392906468, -14.736736062163272, -15.350766731420075, -15.964797400676879];
#DEFINE DRAGS  [-3.6222976277233707, -3.3490177771111052, -3.0864547833855935, -2.8346086465468385, -2.5934793665948392, -2.3630669435295952, -2.1433713773511069, -1.9343926680593737, -1.7361308156543964, -1.5485858201361751, -1.3717576815047086, -1.2056463997599975, -1.0502519749020425, -0.90557440693084268, -0.77161369584639838, -0.64836984164870981, -0.53584284433777674, -0.4340327039135991, -0.34293942037617714, -0.26256299372551062, -0.1929034239615996, -0.13396071108444418, -0.085734855094044285, -0.048225855990399899, -0.021433713773511071, -0.0053584284433777678, -0.0, -0.0053584284433777678, -0.021433713773511071, -0.048225855990399899, -0.085734855094044285, -0.13396071108444418, -0.1929034239615996, -0.26256299372551062, -0.34293942037617714, -0.4340327039135991, -0.53584284433777674, -0.64836984164870981, -0.77161369584639838, -0.90557440693084268, -1.0502519749020425, -1.2056463997599975, -1.3717576815047086, -1.5485858201361751, -1.7361308156543964, -1.9343926680593737, -2.1433713773511069, -2.3630669435295952, -2.5934793665948392, -2.8346086465468385, -3.0864547833855935, -3.3490177771111052, -3.6222976277233707];


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
    std::vector<double> xBoat_Forces [53];  
    std::vector<double> yBoat_Forces [53];
    
    int i;
    // transforming given calculated lifts and drags into forces in
    // the boat coordinates system
    for (i = 0;i < 54;i++)
    {
	xBoat_Forces[i] = cos(DRAGS[i]) - sin (LIFTS[i]);
	yBoat_Forces[i] = cos(LIFTS[i]) + sin (DRAGS[i]);
    }

    list::list maxAndIndex_xBoat_Forces;
    maxAndIndex_xBoat_Forces = Utility::maxAndIndex(xBoat_Forces);

    double orderTail;
    orderTail = maxAndIndex_xBoat_Forces[1] - 26;

    return(orderTail);
    
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
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(0,node->calculateTailAngle);
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));

        // Broadcast() or selected sent???
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
        node->updateFrequencyThread();
    }
}

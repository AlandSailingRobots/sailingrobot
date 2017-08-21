/****************************************************************************************
*
* File:
*       WindStateNode.cpp
*
* Purpose:
*   Each time a vessel state message is received :
*   - Calculates the instantaneous true wind (speed and direction) from wind sensor and Vessel State
*     datas.
*   - Returns a WindStateMsg corresponding to the true and apparent wind state (speed and direction).
*   The wind direction corresponds to the direction where the wind comes from.
*
* Developer Notes:
*
*
***************************************************************************************/

#include "WindStateNode.h"

#define DATA_OUT_OF_RANGE -2000

WindStateNode::WindStateNode(MessageBus& msgBus)
: Node(NodeID::WindStateNode, msgBus)
{
    msgBus.registerNode(*this, MessageType::StateMessage);
    msgBus.registerNode(*this, MessageType::WindData);
}

WindStateNode::~WindStateNode(){}



bool WindStateNode::init()
{
    return true;
}


void WindStateNode::processMessage(const Message* message)
{
    MessageType type = message->messageType();

    if(type == MessageType::StateMessage)
    {
        processVesselStateMessage(dynamic_cast< const StateMessage*> (message));
        WindStateNode::calculateTrueWind();
        sendMessage();
    }
    else if(type == MessageType::WindData)
    {
        processWindMessage(dynamic_cast< const WindDataMsg*> (message));
    }
}

void WindStateNode::processVesselStateMessage(const StateMessage* msg)
{
    m_vesselHeading = msg->heading();
    m_vesselSpeed   = msg->speed();
    m_vesselCourse  = msg->course();
    // std::cout << "m_vesselHeading: " << m_vesselHeading <<std::endl;
}

void WindStateNode::processWindMessage(const WindDataMsg* msg)
{
    m_apparentWindSpeed     = msg->windSpeed();
    m_apparentWindDirection = msg->windDirection();
    // std::cout << "m_apparentWindDirection: " << m_apparentWindDirection <<std::endl;
}

void WindStateNode::sendMessage()
{
    MessagePtr windState = std::make_unique<WindStateMsg>(m_trueWindSpeed, m_trueWindDirection,
        m_apparentWindSpeed, m_apparentWindDirection);
    m_MsgBus.sendMessage(std::move(windState));
}

void WindStateNode::calculateTrueWind()
{
    std::vector<double> v1(2), v2(2), v3(2);

    // v1 = - ApparentWindVector in North-East reference frame
    v1[0] = m_apparentWindSpeed;
    v1[1] = Utility::degreeToRadian(m_vesselHeading + m_apparentWindDirection);

    // v2 = - VelocityVector in North-East reference frame
    v2[0] = - m_vesselSpeed;
    v2[1] = Utility::degreeToRadian(m_vesselCourse);

    // v3 = v1 + v2 (TrueWindVector = ApparentWindVector + VelocityVector)
    v3 = Utility::polarVerctorsAddition(v1, v2);

    // v3 = - TrueWindVector in North-East reference frame
    m_trueWindSpeed = v3[0];
    m_trueWindDirection = Utility::radianToDegree(v3[1]);
}

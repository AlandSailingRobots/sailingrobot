/****************************************************************************************
 *
 * File:
 * 		SailControlNode.cpp
 *
 * Purpose:
 *      Calculates the desired sail angle.
 *      It sends a SailComandMsg corresponding to the command angle of the sail.
 *
 * Developer Notes:
 *      Two functions have been developed to calculate the desired sail angle :
 *          - calculateSailAngleLinear(),
 *          - calculateSailAngleCardioid().
 *      You can choose the one you want to use by commenting/uncommenting lines
 *      in SailControlNodeThreadFunc().
 *
 ***************************************************************************************/

#include "SailControlNode.h"


#define DATA_OUT_OF_RANGE -2000
const int INITIAL_SLEEP = 2000; // milliseconds
const float NO_COMMAND = -1000;


///----------------------------------------------------------------------------------
SailControlNode::SailControlNode(MessageBus& msgBus, DBHandler& dbhandler)
    :ActiveNode(NodeID::SailControlNode,msgBus),m_db(dbhandler), m_Running(0),
    m_LoopTime(0.5), m_MaxSailAngle(90), m_MinSailAngle(10),
    m_ApparentWindDir(DATA_OUT_OF_RANGE)
{
    msgBus.registerNode( *this, MessageType::WindState);
    msgBus.registerNode( *this, MessageType::LocalNavigation);
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived);
}

///----------------------------------------------------------------------------------
SailControlNode::~SailControlNode(){}

///----------------------------------------------------------------------------------
bool SailControlNode::init()
{
    updateConfigsFromDB();
    return true;
}

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
void SailControlNode::processMessage( const Message* msg)
{
    switch( msg->messageType() )
    {
    case MessageType::WindState:
        processWindStateMessage(static_cast< const WindStateMsg*>(msg));
        break;
    case MessageType::ServerConfigsReceived:
        updateConfigsFromDB();
        break;
    default:
        return;
    }
}

///----------------------------------------------------------------------------------
void SailControlNode::updateConfigsFromDB()
{
    m_LoopTime = m_db.tableColumnValueDouble("config_sail_control", "loop_time");
    m_MaxSailAngle = m_db.tableColumnValueInt("config_sail_control", "max_sail_angle");
    m_MinSailAngle = m_db.tableColumnValueInt("config_sail_control", "min_sail_angle");
}

///----------------------------------------------------------------------------------
void SailControlNode::processWindStateMessage(const WindStateMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_ApparentWindDir = msg->apparentWindDirection();
}

///----------------------------------------------------------------------------------
float SailControlNode::restrictSailAngle(float val)
{
    if( val > m_MaxSailAngle)        { return m_MaxSailAngle; }
    else if ( val < m_MinSailAngle) { return m_MinSailAngle; }
    return val;
}

///----------------------------------------------------------------------------------
float SailControlNode::calculateSailAngleLinear()
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    if(m_ApparentWindDir != DATA_OUT_OF_RANGE)
    {
        // Equation from book "Robotic Sailing 2015", page 141
        return (m_MaxSailAngle-m_MinSailAngle)*std::fabs(Utility::limitAngleRange180(m_ApparentWindDir))/180 + m_MinSailAngle; //!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)
    }
    else
    {
        return NO_COMMAND;
    }
}

///----------------------------------------------------------------------------------
float SailControlNode::calculateSailAngleCardioid()
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    if(m_ApparentWindDir != DATA_OUT_OF_RANGE)
    {
        float sailAngle = 90*(-cos(Utility::degreeToRadian(m_ApparentWindDir))+1)/2;
        return restrictSailAngle(sailAngle);
    }
    else
    {
        return NO_COMMAND;
    }
}

///----------------------------------------------------------------------------------
void SailControlNode::SailControlNodeThreadFunc(ActiveNode* nodePtr)
{
    SailControlNode* node = dynamic_cast<SailControlNode*> (nodePtr);

    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(INITIAL_SLEEP));

    Timer timer;
    timer.start();

    while(node->m_Running.load() == true)
    {
        //float sailAngle = node->calculateSailAngleLinear();
        float sailAngle = node->calculateSailAngleCardioid();
        if(sailAngle != NO_COMMAND)
        {
            MessagePtr sailCommandMsg = std::make_unique<SailCommandMsg>(sailAngle);
            node->m_MsgBus.sendMessage(std::move(sailCommandMsg));
        }
        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}

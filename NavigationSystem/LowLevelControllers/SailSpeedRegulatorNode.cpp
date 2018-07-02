/****************************************************************************************
 *
 * File:
 *      SailSpeedRegulatorNode.cpp
 *
 * Purpose:
 *      Calculates the desired sail angle in order to regulate the speed.
 *      It sends a SailComandMsg corresponding to the command angle of the sail.
 *
 * Developer Notes:
 *      This node has not been tested and will probably not work in reality because the speed 
 *      regulator is base on the simulator behaviour which is a rough representation of the reality.
 *
 ***************************************************************************************/

#include "SailSpeedRegulatorNode.h"


#define DATA_OUT_OF_RANGE -2000
const int INITIAL_SLEEP = 2000; // milliseconds
const float NO_COMMAND = -1000;

//SPEED CONTROLLER
#define SPEED_TARGET 1
#define PID_CONTROL 1
#define P_COEF 5
#define D_COEF 1
#define I_COEF 0.5
#define EXP_COEF 20

///----------------------------------------------------------------------------------
SailSpeedRegulatorNode::SailSpeedRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler)
    :ActiveNode(NodeID::SailSpeedRegulatorNode,msgBus),m_db(dbhandler), m_Running(0),
    m_LoopTime(0.5), m_MaxSailAngle(90), m_MinSailAngle(10),
    m_ApparentWindDir(DATA_OUT_OF_RANGE), m_VesselSpeed(DATA_OUT_OF_RANGE), 
    m_old_diff_v(0), m_int_diff_v(0)
{
    msgBus.registerNode( *this, MessageType::WindState);
    msgBus.registerNode( *this, MessageType::StateMessage);
    msgBus.registerNode( *this, MessageType::LocalNavigation);
    msgBus.registerNode( *this, MessageType::ServerConfigsReceived);
}

///----------------------------------------------------------------------------------
SailSpeedRegulatorNode::~SailSpeedRegulatorNode(){}

///----------------------------------------------------------------------------------
bool SailSpeedRegulatorNode::init()
{
    updateConfigsFromDB();
    return true;
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::start()
{
    m_Running.store(true);
    runThread(SailSpeedRegulatorNodeThreadFunc);
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::stop()
{
    m_Running.store(false);
    stopThread(this);
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::processMessage( const Message* msg)
{
    switch( msg->messageType() )
    {
    case MessageType::WindState:
        processWindStateMessage(static_cast< const WindStateMsg*>(msg));
        break;
    case MessageType::StateMessage:
        processStateMessage(static_cast< const StateMessage*>(msg));
        break;
    case MessageType::ServerConfigsReceived:
        updateConfigsFromDB();
        break;
    default:
        return;
    }
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::updateConfigsFromDB()
{
    m_LoopTime = 0.5;//m_db.tableColumnValueDouble("config_sail_control","1","loop_time");
    m_MaxSailAngle = m_db.tableColumnValueInt("config_sail_control", "max_sail_angle", "1");
    m_MinSailAngle = m_db.tableColumnValueInt("config_sail_control", "min_sail_angle", "1");
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::processWindStateMessage(const WindStateMsg* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_ApparentWindDir = msg->apparentWindDirection();
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::processStateMessage(const StateMessage* msg)
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    m_VesselSpeed = msg->speed();
}

///----------------------------------------------------------------------------------
float SailSpeedRegulatorNode::restrictSailAngle(float val)
{
    if( val > m_MaxSailAngle)        { return m_MaxSailAngle; }
    else if ( val < m_MinSailAngle) { return m_MinSailAngle; }
    return val;
}

float SailSpeedRegulatorNode::speedRegulator(float sailCommand)
{
    float diff_v = SPEED_TARGET-m_VesselSpeed;

    float diff_v_dot = (diff_v-m_old_diff_v)/m_LoopTime;
    float int_diff_v=diff_v*m_LoopTime;
    m_old_diff_v = diff_v;

    if (diff_v <= 0)
    {
       sailCommand = 0;
    }
    else
    {
        if (PID_CONTROL)
        {
            float coef_mult =(P_COEF*diff_v/SPEED_TARGET+D_COEF*diff_v_dot+I_COEF*m_int_diff_v);
            if(coef_mult<1 && coef_mult>0)
            {
                m_int_diff_v+=int_diff_v;
            }

            sailCommand = sailCommand*coef_mult;

        }
        else
        {
            sailCommand = sailCommand*exp(-(SPEED_TARGET/diff_v-1)/EXP_COEF);
        }
    }
    return sailCommand;
}

///----------------------------------------------------------------------------------
float SailSpeedRegulatorNode::calculateSailAngleLinear()
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    if(m_ApparentWindDir != DATA_OUT_OF_RANGE && m_VesselSpeed != DATA_OUT_OF_RANGE)
    {
        // Equation from book "Robotic Sailing 2015", page 141
        float sailAngle = (m_MaxSailAngle-m_MinSailAngle)*std::fabs(Utility::limitAngleRange180(m_ApparentWindDir))/180 + m_MinSailAngle; //!!! on some pc abs only ouptut an int (ubuntu 14.04 gcc 4.9.3)
        return speedRegulator(sailAngle);
    }
    else
    {
        return NO_COMMAND;
    }
}

///----------------------------------------------------------------------------------
float SailSpeedRegulatorNode::calculateSailAngleCardioid()
{
    std::lock_guard<std::mutex> lock_guard(m_lock);

    if(m_ApparentWindDir != DATA_OUT_OF_RANGE && m_VesselSpeed != DATA_OUT_OF_RANGE)
    {
        float sailAngle = 90*(-cos(Utility::degreeToRadian(m_ApparentWindDir))+1)/2;
        return speedRegulator(restrictSailAngle(sailAngle));
    }
    else
    {
        return NO_COMMAND;
    }
}

///----------------------------------------------------------------------------------
void SailSpeedRegulatorNode::SailSpeedRegulatorNodeThreadFunc(ActiveNode* nodePtr)
{
    SailSpeedRegulatorNode* node = dynamic_cast<SailSpeedRegulatorNode*> (nodePtr);

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
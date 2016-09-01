/****************************************************************************************
 *
 * File:
 * 		ManualControlNode.cpp
 *
 * Purpose:
 *		ManualControl hardcoded
 *
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "ManualControlNode.h"
#include "Messages/ActuatorPositionMsg.h"
#include "SystemServices/Logger.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define MANUAL_CONTROL_SLEEP_MS 10

ManualControlNode::ManualControlNode(MessageBus& msgBus): ActiveNode(NodeID::ManualControl, msgBus),
                                                          m_Initialised(false)
{

}

bool ManualControlNode::init()
{
    m_Initialised = true;
    m_sailPwm = 1000;
    m_rudderPwm = 1000;
    return m_Initialised;
}

void ManualControlNode::start()
{
    if(m_Initialised)
    {
        runThread(ManualControlThreadFunc);
    }
    else
    {
        Logger::error("%s Cannot start Manual Control thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
    }
}

void ManualControlNode::processMessage(const Message* message){

}

void ManualControlNode::ManualControlThreadFunc(void* nodePtr)
{
    ManualControlNode* node = (ManualControlNode*)nodePtr;

    Logger::info("ManualControl thread started");

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(MANUAL_CONTROL_SLEEP_MS));

        std::cout << "1. sail 2. rudder \n";
        std::string input;
        std::cin >> input;

        std::cout << "Value : ";
        int input2;
        std::cin >> input2;

        if(input=="1"){
            node->m_sailPwm = input2;
        }
        else if(input=="2"){
            node->m_rudderPwm = input2;
        }


        MessagePtr msg = std::make_unique<ActuatorPositionMsg>(node->m_rudderPwm,node->m_sailPwm);
        node->m_MsgBus.sendMessage(std::move(msg));
    }
}
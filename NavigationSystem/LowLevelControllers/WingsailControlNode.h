/****************************************************************************************
 *
 * File:
 * 		WingsailControlNode.h
 *
 * Purpose:
 *      This file realize the control of the wingsail by the wind state.
 *      It sends the direct command to the actuator who control the tail wing angle.
 *
 * Developer Notes:
 *
 ***************************************************************************************/
#pragma once

#include <thread>
#include <math.h>
#include <mutex>
#include <chrono>
#include <vector>
#include <stdint.h>

#include "DataBase/DBHandler.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/WingSailCommandMsg.h"
#include "SystemServices/Timer.h"


class WingsailControlNode : public ActiveNode {
public:

    WingsailControlNode(MessageBus& msgBus, DBHandler& dbhandler);
    ~WingsailControlNode();

    bool init();
    void start();
    void processMessage(const Message* message);

private:

    ///----------------------------------------------------------------------------------
    /// Update values from the database
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Processing informations from the State Message
    ///----------------------------------------------------------------------------------
    void processWindStateMessage(const WindStateMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Processing informations from the Local Navigation Message
    ///----------------------------------------------------------------------------------
    void processLocalNavigationMessage(const LocalNavigationMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Limit the command tail angle to m_MaxCommandAngle
    ///----------------------------------------------------------------------------------
    double restrictWingsail(double val);

    ///----------------------------------------------------------------------------------
    /// Calculate the angle to give to the tail to have maximum force toward boat heading
    ///----------------------------------------------------------------------------------
    float calculateTailAngle();

    ///----------------------------------------------------------------------------------
    /// Set the tail command angle to +/- m_MaxCommandAngle in function of the desired tack of the vessel
    ///----------------------------------------------------------------------------------
    float simpleCalculateTailAngle();

    ///----------------------------------------------------------------------------------
    /// Actions during the activity of the node
    ///----------------------------------------------------------------------------------
    static void WingsailControlNodeThreadFunc(ActiveNode* nodePtr);


    DBHandler &m_db;
    std::mutex m_lock;

    double  m_LoopTime;             // seconds
    double  m_MaxCommandAngle;      // degrees

    double  m_ApparentWindDir;      // degrees [0, 360[ in North-East reference frame (clockwise)
    bool    m_TargetTackStarboard;  // True if the desired tack of the vessel is starboard.
    
};

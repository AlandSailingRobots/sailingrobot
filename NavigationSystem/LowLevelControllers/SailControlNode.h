/****************************************************************************************
 *
 * File:
 * 		SailControlNode.h
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
#pragma once

#include "MessageBus/ActiveNode.h"
#include "Messages/WindStateMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "MessageBus/MessageBus.h"
#include "DataBase/DBHandler.h"
#include <mutex>
#include <stdint.h>


class SailControlNode : public ActiveNode {
public:
    //--------------
    // Constructor
    //--------------
    SailControlNode(MessageBus& msgBus, double loopTime, float maxSailAngle, float minSailAngle
                    float maxCommandAngle, double configPGain, double configIGain, DBHandler& dbhandler)
    
    // -------------
    // Destructor
    // -------------
    ~SailControlNode();

    // -------------
    // Function to init the truth
    // -------------
    bool init();

    // -------------
    // Start the thread for the active node
    // -------------
    void start();

    // -------------
    // Listen the message concerning this Node
    // -------------
    void processMessage(const Message* message);

private:

    // -------------
    // Processing informations from the State Message
    // -------------
    void processWindStateMessage(const WindStateMsg* msg);
        // -------------
        // Informations
        double m_ApparentWindDir; // units : ° (degrees), from -180 to 180

    // -------------
    // Processing informations from the Navigation Control Message
    // -------------
    void processNavigationControlMessage(const NavigationControlMsg* msg);
        // -------------
        // Informations
        NavigationState m_NavigationState;


    // -------------
    // Calculate the sail angle according to the apparent wind
    // -------------    
    double calculateSailAngle();
        // ------------
        // Informations
        double m_MaxSailAngle; // units : ° (degrees)
        double m_MinSailAngle; // units : ° (degrees)

    // -------------
    // Limit the sail angle
    // -------------   
    double restrictSail(double val); 
        // -------------
        // Informations
        double m_MaxCommandAngle; // units : ° (degrees)

    // -------------
    // Actions during the activity of the node
    // -------------
    static void SailControlNodeThreadFunc(ActiveNode* nodePtr);
        // -------------
        // Informations
        std::mutex m_lock;
        double m_LoopTime;
    
    // -------------
    // Access to the database
    // -------------
    DBHandler &m_db;
};


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
#include "Messages/WindDataMsg.h"
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
    SailControlNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime = 0.5, double maxSailAngle = 43, double minSailAngle = 5.5,
                    double maxCommandAngle = 30, double configPGain = 0, double configIGain = 0);

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

    double getFrequencyThread();

private:

    // -------------
    // Processing informations from the State Message
    // -------------
    void processWindDataMessage(const WindDataMsg* msg);

    // -------------
    // Processing informations from the Navigation Control Message
    // -------------
    void processNavigationControlMessage(const NavigationControlMsg* msg);

    // -------------
    // Calculate the sail angle according to the apparent wind
    // -------------
    double calculateSailAngle();

    // -------------
    // Limit the sail angle
    // -------------
    double restrictSail(double val);

    // -------------
    // Get and update the frequency of the thread
    // -------------
    void updateFrequencyThread();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void SailControlNodeThreadFunc(ActiveNode* nodePtr);


    //m_maxCommandAngle = 30;
    //m_maxSailAngle = 42.857;
    //m_minSailAngle = 5.625;
    // ------------
    // Informations
    double m_MaxSailAngle; // units : (radian)
    double m_MinSailAngle; // units : (radian)
    // -------------
    // Informations
    double m_MaxCommandAngle; // units : ° (degrees)

    // -------------
    // Informations
    double m_ApparentWindDir; // units : ° (degrees), from -180 to 180

    double pGain;
    double iGain;

    // -------------
    // Access to the database
    // ------------
    DBHandler &m_db;
    double m_LoopTime;

    const int STATE_INITIAL_SLEEP = 2000;


    // -------------
    // Informations
    //NavigationState m_NavigationState;


    // -------------
    // Informations
    std::mutex m_lock;
};
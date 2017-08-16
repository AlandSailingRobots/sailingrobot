/****************************************************************************************
 *
 * File:
 * 		WingsailControlNode.h
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
#pragma once

#include "MessageBus/ActiveNode.h"
#include "Messages/WindDataMsg.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/WingSailCommandMsg.h"
#include "MessageBus/MessageBus.h"
#include "DataBase/DBHandler.h"
#include <mutex>
#include <stdint.h>


class WingsailControlNode : public ActiveNode {
public:
    //--------------
    // Constructor
    //--------------
    WingsailControlNode(MessageBus& msgBus, DBHandler& dbhandler);

    // -------------
    // Destructor
    // -------------
    ~WingsailControlNode();

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
    void processLocalNavigationMessage(const LocalNavigationMsg* msg);

    // -------------
    // Calculate the sail angle according to the apparent wind
    // -------------
    double calculateWingsailAngle();

    //--------------
    // Calculate the angle to give to the tail
    //
    double calculateTailAngle();

    // -------------
    // Limit the sail angle
    // -------------
    double restrictWingsail(double val);

    // -------------
    // Get and update the frequency of the thread
    // -------------
    void updateFrequencyThread();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void WingsailControlNodeThreadFunc(ActiveNode* nodePtr);


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


    // -------------
    // Informations
    std::mutex m_lock;
};

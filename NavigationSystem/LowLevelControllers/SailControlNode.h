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
#include "Messages/LocalNavigationMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "MessageBus/MessageBus.h"
#include "DataBase/DBHandler.h"
#include <mutex>
#include <stdint.h>
#include <atomic>


class SailControlNode : public ActiveNode {
public:
    //--------------
    // Constructor
    //--------------
    SailControlNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime);

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
    void stop();


    // -------------
    // Listen the message concerning this Node
    // -------------
    void processMessage(const Message* message);

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
    double calculateSailAngle();

    // -------------
    // Limit the sail angle
    // -------------
    double restrictSail(double val);

    ///----------------------------------------------------------------------------------
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void SailControlNodeThreadFunc(ActiveNode* nodePtr);


    //m_maxCommandAngle = 30;
    //m_maxSailAngle = 42.857;
    //m_minSailAngle = 5.625;
    // ------------
    // Informations
    int m_MaxSailAngle; // units : (radian)
    int m_MinSailAngle; // units : (radian)
    // -------------
    // Informations
    double m_MaxCommandAngle; // units : ° (degrees)

    // -------------
    // Informations
    double m_ApparentWindDir; // units : ° (degrees), from -180 to 180

    double pGain;   // without units
    double iGain;   // without units

    // -------------
    // Access to the database
    // ------------
    DBHandler &m_db;
    double m_LoopTime;  //in seconds (ex: 0.5 s)

    // -------------
    // Informations
    //NavigationState m_NavigationState;


    // -------------
    // Informations
    std::mutex m_lock;
    std::atomic<bool> m_Running;
};

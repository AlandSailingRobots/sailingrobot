/****************************************************************************************
 *
 * File:
 * 		CourseRegulatorNode.h
 *
 * Purpose:
 *      This file realize the regulation of the rudder by the desired course.
 *      It sends the value to the rudder actuator node
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/
#pragma once

#include "MessageBus/ActiveNode.h"
#include "Messages/StateMessage.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/DesiredCourseMsg.h"
#include "MessageBus/MessageBus.h"
#include "DataBase/DBHandler.h"
#include <mutex>
#include <stdint.h>

class CourseRegulatorNode : public ActiveNode{
public:
    //--------------
    // Constructor
    //--------------
    CourseRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime = .5, double maxRudderAngle = 30 ,double configPGain = 0, double configIGain = 0);
    // -------------
    // Destructor
    // -------------
    ~CourseRegulatorNode();

    // -------------
    // Function to init the node
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
    void processStateMessage( const StateMessage* msg);

    // -------------
    // Processing informations from the Desired course message
    // -------------
    void processDesiredCourseMessage( const DesiredCourseMsg* msg);

    // -------------
    // Processing informations from the Navigation Control Message
    // -------------
    void processLocalNavigationMessage( const LocalNavigationMsg* msg);

    // -------------
    // Determinate the rudder angle according to the heading difference
    // -------------
    float calculateRudderAngle();

    // -------------
    // Get and update the frequency of the thread
    // -------------
    void updateConfigsFromDB();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr);

    float  m_VesselCourse; // units : ° (degrees), from 0 to 359

    float m_VesselSpeed; // units : m/s
    // -------------
    // Parameters to regulate this node
    // -------------
    float m_MaxRudderAngle; // units :° (degrees), define the extreme value of the rudder
    // -------------
    // Informations
    float m_DesiredCourse; // units : ° (degrees), from 0 to 359
    // -------------
    // Access to the database
    // -------------
    DBHandler &m_db;
    // -------------
    // Loop time where the thread is asleep. units : seconds
    // -------------
    double m_LoopTime;
    // -------------
    // Parameters to make a PI regulation
    // -------------
    float pGain;
    float iGain;

    std::mutex m_lock;

};

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
#include "Messages/NavigationControlMsg.h"
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
    CourseRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime, float maxRudderAngle = 30 ,double configPGain = 0, double configIGain = 0);
    // -------------
    // Destructor
    // -------------
    ~CourseRegulatorNode();

    // -------------
    // Function to init the truth
    // -------------
    bool init();

    // -------------
    // Start the thread for the active node
    // -------------
    void start();

    // -------------
    // Get the frequency of the thread
    // -------------
    double getFrequencyThread();

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
        // Informations
        float  m_VesselHeading; // units : ° (degrees), from 0 to 359
        double m_VesselLatitude;
        double m_VesselLongitude;
        double m_VesselSpeed; // units : knts (knots)
        double m_VesselCourse; // units : ° (degrees), from 0 to 359

    // -------------
    // Processing informations from the Desired course message
    // -------------
    void processDesiredCourseMessage( const DesiredCourseMsg* msg);
        // -------------
        // Informations
        double m_DesiredHeading; // units : ° (degrees), from 0 to 359

    // -------------
    // Processing informations from the Navigation Control Message
    // -------------
    void processNavigationControlMessage( const NavigationControlMsg* msg);
        // -------------
        // Informations on Navigation control message
        NavigationState m_NavigationState;
        int m_CourseToSteer;
        float m_TargetSpeed;
        bool m_Tack = false;

    // -------------
    // Determinate the rudder angle according to the heading difference
    // -------------
    double calculateRudderAngle();

    // -------------
    // Update the frequency of the thread
    // -------------
    void updateFrequencyThread();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr);
        //----------
        // Variable used
        const int STATE_INITIAL_SLEEP = 2000;
        std::mutex m_lock;                      //Mutex to lock the node
        double m_LoopTime;   // Loop time where the thread is asleep. units : seconds

    // -------------
    // Parameters to regulate this node
    // -------------
    double m_MaxRudderAngle; // units :° (degrees), define the extreme value of the rudder
        
    // -------------
    // Parameters to make a PI regulation
    // -------------
    double pGain;
    double iGain;

    // -------------
    // Access to the database
    // -------------
    DBHandler &m_db;
    
};

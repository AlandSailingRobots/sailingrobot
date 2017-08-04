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
    CourseRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime = .5, double maxRudderAngle = 30 ,double configPGain = 0, double configIGain = 0);
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
    // Listen the message concerning this Node
    // -------------
    void processMessage(const Message* message);

    //
    //
    //
    double getFrequencyThread();

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
    void processNavigationControlMessage( const NavigationControlMsg* msg);


    // -------------
    // Determinate the rudder angle according to the heading difference
    // -------------
    double calculateRudderAngle();

    // -------------
    // Get and update the frequency of the thread
    // -------------
    void updateFrequencyThread();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr);

    double  m_VesselHeading; // units : ° (degrees), from 0 to 359

    double m_VesselSpeed; // units : knts (knots)
    // -------------
    // Parameters to regulate this node
    // -------------
    double m_MaxRudderAngle; // units :° (degrees), define the extreme value of the rudder
    // -------------
    // Informations
    double m_DesiredHeading; // units : ° (degrees), from 0 to 359
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
    double pGain;
    double iGain;

    //----------
    // Variable used
    const int STATE_INITIAL_SLEEP = 2000;

    // -------------
    // Informations on Navigation control message
    //NavigationState m_NavigationState;
    /*int m_CourseToSteer;
    float m_TargetSpeed;
    bool m_Tack = false;

     // -------------
    // Informations
    double m_VesselLatitude;
    double m_VesselLongitude;
    double m_VesselCourse; // units : ° (degrees), from 0 to 359
    */
    std::mutex m_lock;                      //Mutex to lock the node

};

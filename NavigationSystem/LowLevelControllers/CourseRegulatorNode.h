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
#include <atomic>

class CourseRegulatorNode : public ActiveNode{
public:
    //--------------
    // Constructor
    //--------------
    CourseRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime);
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

    void stop();

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

    ///----------------------------------------------------------------------------------
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    // -------------
    // Actions during the activity of the node
    // -------------
    static void CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr);

    float m_VesselCourse; // units : ° (degrees), from 0 to 359
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
    double m_LoopTime;      // unit : seconds (ex: 0.5 s)
    // -------------
    // Parameters to make a PI regulation
    // -------------
    double pGain;       //without units
    double iGain;       //without units
    double dGain;       //without units

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
    std::atomic<bool> m_Running;
};

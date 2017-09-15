/****************************************************************************************
 *
 * File:
 * 		CourseRegulatorNode.h
 *
 * Purpose:
 *      Calculates the command angle of the rudder in order to regulate the vessel course.
 *      It sends a RudderCommandMsg corresponding to the command angle of the rudder.
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
#include <atomic>

#include "DataBase/DBHandler.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/StateMessage.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/RudderCommandMsg.h"
#include "SystemServices/Timer.h"


class CourseRegulatorNode : public ActiveNode{
public:
    CourseRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler);
    ~CourseRegulatorNode();

    bool init();
    void start();
    void stop();
    void processMessage(const Message* message);

private:

    ///----------------------------------------------------------------------------------
    /// Updates the values of the parameters from the database
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Stores vessel speed and course datas from a StateMessage.
    ///----------------------------------------------------------------------------------
    void processStateMessage( const StateMessage* msg);

    ///----------------------------------------------------------------------------------
    /// Stores target course data from a LocalNavigationMsg.
    ///----------------------------------------------------------------------------------
    void processLocalNavigationMessage( const LocalNavigationMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Calculates the command rudder angle according to the course difference.
    /// Equation from book "Robotic Sailing 2015 ", page 141.
    ///----------------------------------------------------------------------------------
    float calculateRudderAngle();

    ///----------------------------------------------------------------------------------
    /// Starts the CourseRegulatorNode's thread that pumps out RudderCommandMsg.
    ///----------------------------------------------------------------------------------
    static void CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr);

    DBHandler &m_db;
    std::mutex m_lock;
    std::atomic<bool> m_Running;

    double  m_LoopTime;             // seconds
    double  m_MaxRudderAngle;       // degrees

    double  m_pGain;
    double  m_iGain;
    double  m_dGain;

    float   m_VesselCourse;         // degree [0, 360[ in North-East reference frame (clockwise)
    float   m_VesselSpeed;          // m/s

    float   m_DesiredCourse;        // degree [0, 360[ in North-East reference frame (clockwise)

};

/****************************************************************************************
 *
 * File:
 * 		WingsailControlNode.h
 *
 * Purpose:
 *      Calculates the desired tail wing angle of the wingsail.
 *      It sends a WingSailComandMsg corresponding to the command angle of the tail wing.
 *
 * Developer Notes:
 *      Two functions have been developed to calculate the desired tail angle :
 *          - calculateTailAngle(),
 *          - simpleCalculateTailAngle().
 *      You can choose the one you want to use by commenting/uncommenting lines
 *      in WingsailControlNodeThreadFunc().
 *
 ***************************************************************************************/
#pragma once

#include <math.h>
#include <stdint.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "DataBase/DBHandler.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/MessageBus.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/WindStateMsg.h"
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
    /// Updates the values of the parameters from the database.
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    ///----------------------------------------------------------------------------------
    /// Stores apparent wind direction from a WindStateMsg.
    ///----------------------------------------------------------------------------------
    void processWindStateMessage(const WindStateMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Stores target course and tack data from a LocalNavigationMsg.
    ///----------------------------------------------------------------------------------
    void processLocalNavigationMessage(const LocalNavigationMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Limits the command tail angle to m_MaxCommandAngle.
    ///----------------------------------------------------------------------------------
    double restrictWingsail(double val);

    ///----------------------------------------------------------------------------------
    /// Calculates the angle to give to the tail to have maximum force toward boat heading.
    /// The parameters used by this function have been calculated by CFD simulation. It is
    /// possible that the values of these parameters do not describe the real wing sail behaviour.
    ///----------------------------------------------------------------------------------
    float calculateTailAngle();

    ///----------------------------------------------------------------------------------
    /// Sets the tail command angle to +/- m_MaxCommandAngle in function of the desired tack of the
    /// vessel.
    ///----------------------------------------------------------------------------------
    float simpleCalculateTailAngle();

    ///----------------------------------------------------------------------------------
    /// Starts the WingsailControlNode's thread that pumps out WingSailCommandMsg.
    ///----------------------------------------------------------------------------------
    static void WingsailControlNodeThreadFunc(ActiveNode* nodePtr);

    DBHandler& m_db;
    std::mutex m_lock;

    double m_LoopTime;         // seconds
    double m_MaxCommandAngle;  // degrees

    double m_ApparentWindDir;    // degrees [0, 360[ in North-East reference frame (clockwise)
    float m_TargetCourse;        // degree [0, 360[ in North-East reference frame (clockwise)
    bool m_TargetTackStarboard;  // True if the desired tack of the vessel is starboard.
};

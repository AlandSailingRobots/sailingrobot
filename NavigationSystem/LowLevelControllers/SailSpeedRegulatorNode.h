/****************************************************************************************
 *
 * File:
 *      SailSpeedRegulatorNode.h
 *
 * Purpose:
 *      Calculates the desired sail angle in order to regulate the speed.
 *      It sends a SailComandMsg corresponding to the command angle of the sail.
 *
 * Developer Notes:
 *      This node has not been tested and will probably not work in reality because the speed
 *      regulator is base on the simulator behaviour which is a rough representation of the reality.
 *
 ***************************************************************************************/
#pragma once

#include <math.h>
#include <stdint.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "../Database/DBHandler.h"
#include "../Math/Utility.h"
#include "../MessageBus/ActiveNode.h"
#include "../MessageBus/MessageBus.h"
#include "../Messages/SailCommandMsg.h"
#include "../Messages/StateMessage.h"
#include "../Messages/WindDataMsg.h"
#include "../SystemServices/Timer.h"

class SailSpeedRegulatorNode : public ActiveNode {
   public:
    SailSpeedRegulatorNode(MessageBus& msgBus, DBHandler& dbhandler);
    ~SailSpeedRegulatorNode();

    bool init();
    void start();
    void stop();
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
    /// Stores vessel speed and course datas from a StateMessage.
    ///----------------------------------------------------------------------------------
    void processStateMessage(const StateMessage* msg);

    ///----------------------------------------------------------------------------------
    /// Limits the command sail angle between m_MaxSailAngle and m_MinSailAngle.
    ///----------------------------------------------------------------------------------
    float restrictSailAngle(float val);

    float speedRegulator(float val);

    ///----------------------------------------------------------------------------------
    /// Calculate the sail angle according to a linear relation to the apparent wind direction.
    ///----------------------------------------------------------------------------------
    float calculateSailAngleLinear();

    ///----------------------------------------------------------------------------------
    /// Calculate the sail angle according to a cardioid relation to the apparent wind direction.
    ///----------------------------------------------------------------------------------
    float calculateSailAngleCardioid();

    ///----------------------------------------------------------------------------------
    /// Starts the SailSpeedRegulatorNode's thread that pumps out SailCommandMsg.
    ///----------------------------------------------------------------------------------
    static void SailSpeedRegulatorNodeThreadFunc(ActiveNode* nodePtr);

    DBHandler& m_db;
    std::mutex m_lock;
    std::atomic<bool> m_Running;

    double m_LoopTime;      // seconds
    double m_MaxSailAngle;  // degrees
    double m_MinSailAngle;  // degrees

    double m_ApparentWindDir;  // degrees [0, 360[ in North-East reference frame (clockwise)
    float m_VesselSpeed;       // m/s

    float m_old_diff_v;  // m/s
    float m_int_diff_v;
};

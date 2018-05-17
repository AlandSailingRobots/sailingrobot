/****************************************************************************************
 *
 * File:
 *       WindStateNode.h
 *
 * Purpose:
 *   Each time a vessel state message is received :
 *   - Calculates the instantaneous true wind (speed and direction) from wind sensor and Vessel
 *State datas.
 *   - Returns a WindStateMsg corresponding to the true and apparent wind state (speed and
 *direction). The wind direction corresponds to the direction where the wind comes from.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include <mutex>
#include <vector>

#include "DataBase/DBHandler.h"
#include "Math/Utility.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/MessageTypes.h"
#include "Messages/StateMessage.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WindStateMsg.h"

class WindStateNode : public Node {
   public:
    WindStateNode(MessageBus& msgBus);
    ~WindStateNode();

    bool init();
    void processMessage(const Message* message);

   private:
    ///----------------------------------------------------------------------------------
    /// Stores vessel state datas from a StateMessage.
    ///----------------------------------------------------------------------------------
    void processVesselStateMessage(const StateMessage* msg);

    ///----------------------------------------------------------------------------------
    /// Stores apparent wind datas from a WindDataMsg.
    ///----------------------------------------------------------------------------------
    void processWindMessage(const WindDataMsg* msg);

    ///----------------------------------------------------------------------------------
    /// Sends windStateMsg.
    ///----------------------------------------------------------------------------------
    void sendMessage();

    ///----------------------------------------------------------------------------------
    /// Calculates the instantaneous true wind (speed and direction).
    ///----------------------------------------------------------------------------------
    void calculateTrueWind();

    float m_vesselHeading;          // degree [0, 360[ in North-East reference frame (clockwise)
    float m_vesselSpeed;            // m/s
    float m_vesselCourse;           // degree [0, 360[ in  North-East reference frame (clockwise)
    float m_apparentWindSpeed;      // m/s
    float m_apparentWindDirection;  // degree [0, 360[ in vessel reference frame (clockwise)

    float m_trueWindSpeed;      // m/s
    float m_trueWindDirection;  // degree [0, 360[ in North-East reference frame (clockwise)
};

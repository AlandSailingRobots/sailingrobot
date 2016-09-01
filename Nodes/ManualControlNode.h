/****************************************************************************************
 *
 * File:
 * 		ManualControlNode.h
 *
 * Purpose:
 *		ManualControl hardcoded.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "ActiveNode.h"

class ManualControlNode : public ActiveNode {
public:
    ManualControlNode(MessageBus& msgBus);

    ///----------------------------------------------------------------------------------
    /// Attempts to connect to the Arduino.
    ///----------------------------------------------------------------------------------
    bool init();

    ///----------------------------------------------------------------------------------
    /// This function should be used to start the active nodes thread.
    ///----------------------------------------------------------------------------------
    void start();

    void processMessage(const Message* message);

private:



    static void ManualControlThreadFunc(void* nodePtr);

    bool 			m_Initialised;
    double          m_time;
    int             m_sailPwm;
    int             m_rudderPwm;
};

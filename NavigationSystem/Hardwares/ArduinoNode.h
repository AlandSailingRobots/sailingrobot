/****************************************************************************************
 *
 * File:
 * 		ArduinoNode.h
 *
 * Purpose:
 *		The Arduino node communicates with the arduino. Sends data about the pressure, rudder, sheet
 *and battery.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Database/DBHandler.h"
#include "Hardwares/i2ccontroller/I2CController.h"
#include "MessageBus/ActiveNode.h"

class ArduinoNode : public ActiveNode {
   public:
    ArduinoNode(MessageBus& msgBus, DBHandler& dbhandler);

    ///----------------------------------------------------------------------------------
    /// Attempts to connect to the Arduino.
    ///----------------------------------------------------------------------------------
    bool init();

    ///----------------------------------------------------------------------------------
    /// This function should be used to start the active nodes thread.
    ///----------------------------------------------------------------------------------
    void start();

    void processMessage(const Message* msg);

   private:
    void updateConfigsFromDB();

    static void ArduinoThreadFunc(ActiveNode* nodePtr);

    I2CController m_I2C;
    bool m_Initialised;
    int m_pressure;
    int m_rudder;
    int m_sheet;
    int m_battery;
    int m_RC;
    double m_LoopTime;
    DBHandler& m_db;
};

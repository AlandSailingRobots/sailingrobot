/****************************************************************************************
*
* File:
*       VelvetWindSensorSerialNode.cpp
*
* Purpose:
*        Process messages from the serial connection to the Arduino
*
* Developer Notes:
*        The arduino on the velvet is sending two different data as for now:
*        windsensor and compass.
*
***************************************************************************************/
#pragma once

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "../Database/DBHandler.h"
#include "../MessageBus/ActiveNode.h"
#include "../MessageBus/Message.h"
#include "../MessageBus/MessageBus.h"
#include "../Messages/WindDataMsg.h"
#include "../SystemServices/Timer.h"
#include "../Hardwares/Serial_Connection/serial_interface.h"

class VelvetWindSensorSerialNode : public ActiveNode {
   public:
    VelvetWindSensorSerialNode(MessageBus& messageBus, DBHandler& dbhandler);
    ~VelvetWindSensorSerialNode();
    bool init();
    void processMessage(const Message* message);
    void start();

   private:
    static void VelvetWindSensorSerialNodeThreadFunc(ActiveNode* nodePtr);

    ///----------------------------------------------------------------------------------
    /// Update values from the database as the loop time of the thread and others parameters
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    float m_apparentWindDir;          // in degree
    
    double m_LoopTime;  // in seconds (ex: 0.5 s)
    DBHandler& m_db;

    std::mutex m_lock;
};

/****************************************************************************************
 *
 * File:
 * 		CANCurrentSensorNode.h
 *
 * Purpose:
 *		 Process messages from current sensors via the CAN-Service .
 *
 * Developer Notes:
 *		 The CAN frame id numbers that this node subscribes to are:
 *			600.
 *
 ***************************************************************************************/
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "../Database/DBHandler.h"
#include "../Hardwares/CAN_Services/CANFrameReceiver.h"
#include "../Hardwares/CAN_Services/CANService.h"
#include "../MessageBus/ActiveNode.h"
#include "../MessageBus/Message.h"
#include "../MessageBus/MessageBus.h"
#include "../Messages/CurrentSensorDataMsg.h"
#include "../SystemServices/Timer.h"

class CANCurrentSensorNode : public ActiveNode, public CANFrameReceiver {
   public:
    CANCurrentSensorNode(MessageBus& messageBus, DBHandler& dbhandler, CANService& canService);
    ~CANCurrentSensorNode();
    bool init();
    void processMessage(const Message* message);
    void processFrame(CanMsg& msg);
    void start();

   private:
    static void CANCurrentSensorNodeThreadFunc(ActiveNode* nodePtr);

    ///----------------------------------------------------------------------------------
    /// Update values from the database as the loop time of the thread and others parameters
    ///----------------------------------------------------------------------------------
    void updateConfigsFromDB();

    float m_current;          // in mA
    float m_voltage;          // in mV
    SensedElement m_element;  // the element measured

    double m_LoopTime;  // in seconds (ex: 0.5 s)
    DBHandler& m_db;

    std::mutex m_lock;
};

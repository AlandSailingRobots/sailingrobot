/****************************************************************************************
 *
 * File:
 * 		CANSolarTrackerNode.h
 *
 * Purpose:
 *		Sends the position, heading and time to the solar tracker
 *
 *
 * Developer Notes:
 *		As of right now, only unit test exist, uncertain if it should work like this
 *		Since we don't have the solartracker yet I dont know exactly what messages to
 *		send but it is easily changed later
 *
 ***************************************************************************************/
#pragma once

#include "Database/DBHandler.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Timer.h"

#include <time.h>
#include <cstring>
#include <iostream>
#include <mutex>
#include <vector>

class CANSolarTrackerNode : public ActiveNode {
   public:
    /*
     * Constructor, pointer to a message bus and canservice
     * double loopTime, how often we send messages
     */
    CANSolarTrackerNode(MessageBus& msgBus,
                        DBHandler& dbhandler,
                        CANService& canService,
                        double loopTime);
    ~CANSolarTrackerNode();

    bool init();

    /*
     * Processes the messages we receive
     */
    void processMessage(const Message* message);

    /*
     * Processes the can messages we receive
     */
    void processFrame(CanMsg& Msg);

    /*
     * Sends message on the canbus
     */
    void sendMsg(float lat, float lon, float head, uint16_t h, uint16_t m);

    /*
     * Starts the worker thread
     */
    void start();

   private:
    /*
     * Update values from the database as the loop time of the thread and others parameters
     */
    void updateConfigsFromDB();

    /*
     * The function that the thread works on
     */
    static void CANSolarTrackerThreadFunc(ActiveNode* nodePtr);

    /*
     * Private variables
     */
    CANService* m_CANService;
    float m_Lat;
    float m_Lon;
    uint16_t m_Hour;
    uint16_t m_Minute;
    float m_Heading;
    bool m_initialised;
    double m_LoopTime;
    DBHandler& m_db;

    std::mutex m_lock;
};

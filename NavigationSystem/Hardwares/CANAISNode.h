/****************************************************************************************
 *
 * File:
 * 		CANAISNode.h
 *
 * Purpose:
 *		Receives the NMEA 2000 messages and picks out the important bits (MMSI, position, heading,
 *speed, length and width) and sends it to AISProcessing
 *
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/
#pragma once

#include "Database/DBHandler.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "Math/Utility.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "Messages/AISDataMsg.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

class CANAISNode : public CANPGNReceiver, public ActiveNode {
   public:
    /*
     * Constructor, pointer to a message bus and canservice
     * double loopTime, how often we send messages
     */
    CANAISNode(MessageBus& msgBus, DBHandler& dbhandler, CANService& canService);
    ~CANAISNode();

    bool init();

    /*
     * Processes any message we receive from the messagebus
     */
    void processMessage(const Message* message);

    /*
     * Receives the N2kMsg and processes it
     */
    void processPGN(N2kMsg& nMsg);

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
     * Class A or B position report
     * Processes and return the data of the message
     * if the PGN is 129038 or 129039
     */
    void parsePGN129038_129039(N2kMsg& nMsg);

    /*
     * GPS position, rapid update
     * Processes and return the data of the message
     * if the PGN is 129025
     */
    void parsePGN129025(N2kMsg& nMsg);

    /*
     * AIS Class A static and voyage related data
     * Processes and return the data of the message
     * if the PGN is 129794
     */
    void parsePGN129794(N2kMsg& nMsg);

    /*
     * AIS Class B static data part B
     * Processes and return the data of the message
     * if the PGN is 129810
     */
    void parsePGN129810(N2kMsg& nMsg);

    /*
     * The function that the thread works on
     */
    static void CANAISThreadFunc(ActiveNode* nodePtr);

    /*
     * Private variables
     */
    std::vector<AISVessel> m_VesselList;
    std::vector<AISVesselInfo> m_VesselInfoList;
    double m_PosLat;
    double m_PosLon;
    std::mutex m_lock;
    double m_LoopTime;
    DBHandler& m_db;
};

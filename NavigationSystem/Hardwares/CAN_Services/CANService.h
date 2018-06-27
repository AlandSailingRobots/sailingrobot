/****************************************************************************************
 *
 * File:
 * 		CANService.h
 *
 * Purpose:
 *		Handles communcation between CAN-Receivers on the system
 *
 *
 * Developer Notes:
 *
 *
 *
 *
 *
 ***************************************************************************************/

#pragma once

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>
#include "../SystemServices/Logger.h"
#include "CANFrameReceiver.h"
#include "CANPGNReceiver.h"
#include "N2kMsg.h"
#include "mcp2515.h"

struct FastPKGInfo {
    N2kMsg n2kmsg;
    int bytesLeft;
    uint8_t latestSeqnumber;
};

class CANService {
   public:
    CANService() {}

    ~CANService() {}

    /*  Registers a CAN receiver with an associated PGN-number     *
     *  which will receive any message with that number sent into  *
     *  the CAN-Service                                            */
    bool registerForReading(CANPGNReceiver& receiver, uint32_t PGN);

    bool registerForReading(CANFrameReceiver& receiver, uint32_t ID);

    /* Sends a NMEA2000 message onto the service, which will  *
     * then either be sent to another receiver, or if no such *
     * receiver is registered, will be discarded.             */
    void sendCANMessage(CanMsg& msg);

    CanMsg getCANMessage();

    /* Starts the service using async, and will begin *
     * receiving and sending messages                 */
    std::future<void> start();

    void SetLoopBackMode();

    void SetNormalMode();

    bool checkMissedMessages();

    /* Stops the service */
    void stop();

   private:
    /* Starts the CANService */
    void run();

    /* Recieves and parses the fast messages and stores everything *
     * in the N2kMsg                                               */
    bool ParseFastPkg(CanMsg& msg, N2kMsg& nMsg);

    /* Checks if the message is a NMEA2000 fast package */
    bool IsFastPackage(const N2kMsg& nMsg);

    /* Private variables */

    std::map<uint32_t, CANPGNReceiver*> m_RegisteredPGNReceivers;
    std::map<uint32_t, CANFrameReceiver*> m_RegisteredFrameReceivers;
    std::map<IDsID, FastPKGInfo> FastPackages;
    std::queue<CanMsg> m_MsgQueue;
    std::mutex m_QueueMutex;

    std::atomic<bool> m_Running;
};

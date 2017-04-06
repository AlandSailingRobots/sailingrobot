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

 #include "CANPGNReceiver.h"
 #include "N2kMsg.h"
 #include <vector>
 #include <map>
 #include <mutex>
 #include <queue>
 #include <memory>
 #include <future>
 #include <atomic>


class CANService {
public:
  CANService() {}

  ~CANService() {}

/*  Registers a CAN receiver with an associated PGN-number     *
 *  which will receive any message with that number sent into  *
 *  the CAN-Service                                            */
  bool registerForReading(CANPGNReceiver& receiver, uint32_t PGN);

/* Sends a NMEA2000 message onto the service, which will  *
 * then either be sent to another receiver, or if no such *
 * receiver is registered, will be discarded.             */
  void sendN2kMessage(N2kMsg& msg);

/* Starts the service using async, and will begin *
 * receiving and sending messages                 */
  std::future<void> start();

  /* Stops the service */
  void stop();

private:

  void run();

  std::map<uint32_t, CANPGNReceiver*> m_RegisteredReceivers;
  std::queue<N2kMsg> m_MsgQueue;
  std::mutex m_QueueMutex;
  std::atomic<bool> m_Running;
};

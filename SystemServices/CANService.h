/****************************************************************************************
 *
 * File:
 * 		CANService.h
 *
 * Purpose:
 *		Handles communcation between CAN-Nodes on the system
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
 #include <vector>
 #include <map>
 #include <mutex>
 #include <queue>
 #include <memory>
 #include <future>

// Temporarily included in this file while coding the service
 struct N2kMsg
 {
 	uint32_t PGN;
 	uint8_t Priority;
 	uint8_t Source;
 	uint8_t Destination;
 	int DataLen;
 	std::vector<uint8_t> Data;
 };


class CANService {
public:
  CANService();

  ~CANService();

  bool registerForReading(CanPGNReceiver& node, uint32_t PGN);

  void sendN2kMessage(N2kMsg& msg);

  void start();

private:

  void run();

  std::map<uint32_t, CanPGNReceiver*> m_RegisteredNodes;
  std::queue<N2kMsg> m_MsgQueue;
  std::mutex m_QueueMutex;
  bool m_Running;
};

#include "CANService.h"

#include <chrono>
#include <thread>

#define SLEEP_TIME_MS 50

bool CANService::registerForReading(CANPGNReceiver& node, uint32_t PGN) {

  // If the PGN is already registered, do not re-register.
  // Perhaps this should be changed in the future.

  if(m_RegisteredNodes.find(PGN) == m_RegisteredNodes.end()){
    return false;
  }

  m_RegisteredNodes[PGN] = &node;
  return true;

}

void CANService::sendN2kMessage(N2kMsg& msg){
  std::lock_guard<std::mutex> lock (m_QueueMutex);
  m_MsgQueue.push(msg);
}

void CANService::start() {
  m_Running = true;
  std::async(&CANService::run, this);
}

void CANService::run() {
  while(m_Running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));

    if(m_MsgQueue.size() > 0) {

      N2kMsg msg = m_MsgQueue.front();
      m_MsgQueue.pop();
      auto nodeIt = m_RegisteredNodes.find(msg.PGN);

      if(nodeIt != m_RegisteredNodes.end()) {

        // Iterator is a pair, of which the second element is the actual node.
        CANPGNReceiver* node = nodeIt->second;
        node->processPGN(msg.Data, msg.PGN);
      }
    }
  }
}

#include "CANService.h"

#include <chrono>
#include <thread>
#include <future>

#define SLEEP_TIME_MS 50

bool CANService::registerForReading(CANPGNReceiver& receiver, uint32_t PGN) {

  // If the PGN is already registered, do not re-register.
  // Perhaps this should be changed in the future.

  if(m_RegisteredReceivers.find(PGN) != m_RegisteredReceivers.end()){
    return false;
  }
  m_RegisteredReceivers[PGN] = &receiver;
  return true;

}

void CANService::sendN2kMessage(N2kMsg& msg){
  std::lock_guard<std::mutex> lock (m_QueueMutex);
  m_MsgQueue.push(msg);
}

std::future<void> CANService::start() {
  m_Running.store(true);
  return std::async(std::launch::async, &CANService::run, this);
}

void CANService::run() {
  while(m_Running.load() == true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));

    if(m_MsgQueue.size() > 0) {
      N2kMsg msg = m_MsgQueue.front();
      m_MsgQueue.pop();
      auto receiverIt = m_RegisteredReceivers.find(msg.PGN);

      if(receiverIt != m_RegisteredReceivers.end()) {
        // Iterator is a pair, of which the second element is the actual receiver.
        CANPGNReceiver* receiver = receiverIt->second;
        receiver->processPGN(msg);
      }
    }
  }
}

void CANService::stop() {
  m_Running.store(false);
}

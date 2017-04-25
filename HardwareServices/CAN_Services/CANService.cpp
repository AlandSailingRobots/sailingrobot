#include "CANService.h"
#include "global.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <future>

#define SLEEP_TIME_MS 50

bool CANService::registerForReading(CANPGNReceiver& receiver, uint32_t PGN) {

  // If the PGN is already registered, do not re-register.
  // Perhaps this should be changed in the future.

  if(m_RegisteredPGNReceivers.find(PGN) != m_RegisteredPGNReceivers.end()){
    return false;
  }
  m_RegisteredPGNReceivers[PGN] = &receiver;
  return true;

}

bool CANService::registerForReading(CANFrameReceiver& receiver, uint32_t ID) {

  // If the ID is already registered, do not re-register.
  // Perhaps this should be changed in the future.

  if(m_RegisteredFrameReceivers.find(ID) != m_RegisteredFrameReceivers.end()){
    return false;
  }
  m_RegisteredFrameReceivers[ID] = &receiver;
  return true;

}

void CANService::sendN2kMessage(N2kMsg& msg){
  std::lock_guard<std::mutex> lock (m_QueueMutex2);
  m_MsgQueue2.push(msg);
}

void CANService::sendCANMessage(CanMsg& msg){
  std::lock_guard<std::mutex> lock (m_QueueMutex);
  m_MsgQueue.push(msg);
}

std::future<void> CANService::start() {
  m_Running.store(true);
  wiringPiSetup();
  int SPISpeed = 1000000;

	//pinMode(MCP2515_INT, INPUT);					//set the interrupt pin to input
	if(wiringPiSPISetup(CHANNEL, SPISpeed) == -1)	//channel, SPI speed
	{
		std::cout << "Could not setup wiring pi" << std::endl;
	}

  bool mcp_initialized = MCP2515_Init();
  if(!mcp_initialized) {
    std::cout << "Could not initialize hardware" << std::endl;
  }
  return std::async(std::launch::async, &CANService::run, this);
}

void CANService::run() {
  while(m_Running.load() == true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));

    static CanMsg Cmsg;

    if(MCP2515_GetMessage(&Cmsg,0)) {

      //N2kMsg msg = m_MsgQueue.front();
      //m_MsgQueue.pop();

//all this can become just one function if processPGN changes to send both CanMsg and N2KMsg 
      if(Cmsg.header.ide == 1) {

        N2kMsg Nmsg;

        IdToN2kMsg(Nmsg,Cmsg.id);

        Nmsg.DataLen = Cmsg.header.length;		//Single frame message
        Nmsg.Data.resize(Cmsg.header.length);
        for(int i = 0; i < 8; ++i)
        {
          Nmsg.Data[i] = Cmsg.data[i];
        }

        auto receiverIt = m_RegisteredPGNReceivers.find(Nmsg.PGN);

        if(receiverIt != m_RegisteredPGNReceivers.end()) {
          // Iterator is a pair, of which the second element is the actual receiver.
          CANPGNReceiver* receiver = receiverIt->second;
          receiver->processPGN(Nmsg);
        }
      }

      else if(Cmsg.header.ide == 0) {

        N2kMsg Nmsg;

        IdToN2kMsg(Nmsg,Cmsg.id);

        auto receiverIt = m_RegisteredFrameReceivers.find(Nmsg.PGN);

        if(receiverIt != m_RegisteredFrameReceivers.end()) {
          // Iterator is a pair, of which the second element is the actual receiver.
          CANFrameReceiver* receiver = receiverIt->second;
          receiver->processPGN(Cmsg);
        }

      }

      else {
        std::cout << "Error: Cmsg.header.ide = " << Cmsg.header.ide;
        std::cout << " - should be 0 or 1" << std::endl;
      }

      
    }

    if(!m_MsgQueue.empty()) {

      CanMsg Cmsg = m_MsgQueue.front();
      m_MsgQueue.pop();

      N2kMsg Nmsg;

      IdToN2kMsg(Nmsg,Cmsg.id);

      auto receiverIt = m_RegisteredFrameReceivers.find(Nmsg.PGN);

      if(receiverIt != m_RegisteredFrameReceivers.end()) {
        // Iterator is a pair, of which the second element is the actual receiver.
        CANFrameReceiver* receiver = receiverIt->second;
        receiver->processPGN(Cmsg);
      }
    }

    if(!m_MsgQueue2.empty()) {

      N2kMsg Nmsg = m_MsgQueue2.front();
      m_MsgQueue2.pop();

      auto receiverIt = m_RegisteredPGNReceivers.find(Nmsg.PGN);

      if(receiverIt != m_RegisteredPGNReceivers.end()) {
        // Iterator is a pair, of which the second element is the actual receiver.
        CANPGNReceiver* receiver = receiverIt->second;
        receiver->processPGN(Nmsg);
      }
    }
  }
}

void CANService::stop() {
  m_Running.store(false);
}


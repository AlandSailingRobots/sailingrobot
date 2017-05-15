#include "CANService.h"
#include "global.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <future>

#define SLEEP_TIME_MS 50

bool CANService::registerForReading(CANPGNReceiver& receiver, uint32_t PGN)
{

  // If the PGN is already registered, do not re-register.
  // Perhaps this should be changed in the future.

  if(m_RegisteredPGNReceivers.find(PGN) != m_RegisteredPGNReceivers.end())
  {
    return false;
  }
  m_RegisteredPGNReceivers[PGN] = &receiver;
  return true;

}

bool CANService::registerForReading(CANFrameReceiver& receiver, uint32_t ID)
{

  // If the ID is already registered, do not re-register.
  // Perhaps this should be changed in the future.

  if(m_RegisteredFrameReceivers.find(ID) != m_RegisteredFrameReceivers.end())
  {
    return false;
  }
  m_RegisteredFrameReceivers[ID] = &receiver;
  return true;

}

void CANService::sendCANMessage(CanMsg& msg)
{
  std::lock_guard<std::mutex> lock (m_QueueMutex);
  m_MsgQueue.push(msg);
}

CanMsg CANService::getCANMessage()
{
  std::lock_guard<std::mutex> lock (m_QueueMutex);
  CanMsg Cmsg = m_MsgQueue.front();
  m_MsgQueue.pop();
  return Cmsg;
}

std::future<void> CANService::start() 
{
  m_Running.store(true);
  wiringPiSetup();
  int SPISpeed = 1000000;

	//pinMode(MCP2515_INT, INPUT);					//set the interrupt pin to input
	if(wiringPiSPISetup(CHANNEL, SPISpeed) == -1)	//channel, SPI speed
	{
		std::cout << "Could not setup wiring pi" << std::endl;
	}

  bool mcp_initialized = MCP2515_Init();
  if(!mcp_initialized) 
  {
    std::cout << "Could not initialize hardware" << std::endl;
  } 
  return std::async(std::launch::async, &CANService::run, this);
}

void CANService::run() 
{
  while(m_Running.load() == true) 
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));

    static CanMsg Cmsg;

    if(MCP2515_GetMessage(&Cmsg,0))
    {
      if(Cmsg.header.ide == 1)
      {
        N2kMsg Nmsg;

        CanMsgToN2kMsg(Cmsg, Nmsg);

        auto receiverIt = m_RegisteredPGNReceivers.find(Nmsg.PGN);

        if(receiverIt != m_RegisteredPGNReceivers.end())
        {  // Iterator is a pair, of which the second element is the actual receiver.
          CANPGNReceiver* receiver = receiverIt->second;
          receiver->processPGN(Nmsg);
        }
      }

      else if(Cmsg.header.ide == 0)
      {
        auto receiverIt = m_RegisteredFrameReceivers.find(Cmsg.id);

        if(receiverIt != m_RegisteredFrameReceivers.end())
        { // Iterator is a pair, of which the second element is the actual receiver.
          CANFrameReceiver* receiver = receiverIt->second;
          receiver->processFrame(Cmsg);
        }
      }
      else
      {
        std::cout << "Error: Cmsg.header.ide = " << Cmsg.header.ide;
        std::cout << " - should be 0 or 1" << std::endl;
      }
    }

    if(!m_MsgQueue.empty())
    {
      CanMsg CmsgSend = getCANMessage();
      MCP2515_SendMessage(&CmsgSend, 0);
    }
  }
}

void CANService::SetLoopBackMode()
{
	uint8_t Mode = 0x40;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}

void CANService::SetNormalMode()
{
	uint8_t Mode = 0x00;
	uint8_t Mask = (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0);
	MCP2515_BitModify(CANCTRL, Mask, Mode);
	uint8_t Status = MCP2515_Read(CANSTAT);	//wait untill it changes
	while((Status & Mask) != Mode)
	{
		Status = MCP2515_Read(CANSTAT);
	}
}

bool CANService::checkMissedMessages()
{
  uint8_t EF = MCP2515_Read(EFLG);
  if(((EF>>7)&1) || ((EF>>6)&1)) return true;
  else return false;
}

void CANService::stop()
{
  m_Running.store(false);
}


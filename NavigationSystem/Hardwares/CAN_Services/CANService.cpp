#include "CANService.h"
#include "global.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <stdlib.h>

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
  int SPISpeed = 5000000;

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
  bool done;
  N2kMsg Nmsg;
  while(m_Running.load() == true)
  {
    static CanMsg Cmsg;

    if(MCP2515_GetMessage(&Cmsg,0))
    {
      if(Cmsg.header.ide == 1)
      {
        done = false;
        IdToN2kMsg(Nmsg, Cmsg.id);
        if (IsFastPackage(Nmsg)) {
          done = ParseFastPkg(Cmsg, Nmsg);
        }
        else {
          CanMsgToN2kMsg(Cmsg, Nmsg);
          done = true;
        }
        auto receiverIt = m_RegisteredPGNReceivers.find(Nmsg.PGN);

        if (done) {
          if(receiverIt != m_RegisteredPGNReceivers.end())
          {  // Iterator is a pair, of which the second element is the actual receiver.
            CANPGNReceiver* receiver = receiverIt->second;
            receiver->processPGN(Nmsg);
          }
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

bool CANService::ParseFastPkg(CanMsg& msg, N2kMsg& nMsg) {
  /*
  * More in-depth about fast packages:
  * All messages have a data part 8 bytes long but some of it is used to ID the message
  * The first message has message ID and sequnce ID and Number in byte 0
  * and how many bytes the entire fast package is in byte 1
  * Bytes 2-7 is the actual data
  * The rest of the messages will have the message ID and Sequnce number and ID in byte 0
  * and the rest of the bytes (1-7) will contain the data!
  */

	IDsID Key = IDsID(msg.id, msg.data[0]&0xE0);		//message ID and sequence ID
	uint8_t SequenceNumber = msg.data[0]&0x1F;

	auto it = FastPackages.find(Key);

  if(SequenceNumber != 0) {				//have parts of the message already
		int LastByte;
    if (it == FastPackages.end()) { // Not in the map, even though SequenceNumber > 0 -> message received in wrong order
      return false;
    } else if (FastPackages[Key].latestSeqnumber+1 != SequenceNumber) { // Messages received in wrong order, remove the key entirely to avoid memory issues (It will get resent later)
      FastPackages.erase(Key);
      return false;
    }
		if(it->second.bytesLeft >= 7)	{		//check if less than 7 bytes left, 1st byte is the sequence ID and Sequence number followed by 7 bytes of data
			LastByte = 8;
		}
		else {
			LastByte = it->second.bytesLeft +1;
		}

		for(int i = 1; i < LastByte; ++i) {
			FastPackages[Key].n2kmsg.Data[6+(SequenceNumber-1)*7 +i-1] = msg.data[i];
		}
		it->second.bytesLeft -= 7;				//decrease bytes left
    it->second.latestSeqnumber = SequenceNumber;
		if(it->second.bytesLeft <= 0) {	//have the whole message, erase key and return true
			nMsg = FastPackages[Key].n2kmsg;
			FastPackages.erase(Key);
			return true;
		}
		else { // Still bytes left to be received from another message
			return false;
		}
	}
	else {        //not found, create new N2kMsg
		uint8_t BytesInMsg = msg.data[1];
		nMsg.DataLen = BytesInMsg;
		nMsg.Data.resize(BytesInMsg);
		for(int i = 2; i < 8; ++i) {
			nMsg.Data[i-2] = msg.data[i];
		}
		if(BytesInMsg <= 6) {
			return true;
		}
		else {
			FastPackages[Key].bytesLeft = BytesInMsg-6;		//how many bytes left
      FastPackages[Key].latestSeqnumber = SequenceNumber;
			FastPackages[Key].n2kmsg = nMsg;
			return false;
		}
	}
}

bool CANService::IsFastPackage(const N2kMsg &nMsg) {
	switch(nMsg.PGN) {
		case 126464: return true;
		case 126996: return true;
		case 65240: return true;
		case 126208: return true;
		case 129038: return true;
		case 129039: return true;
		case 129041: return true;
		case 129793: return true;
		case 129794: return true;
		case 129798: return true;
		case 129802: return true;
		case 129809: return true;
		case 129810: return true;
	}
	return false;
}

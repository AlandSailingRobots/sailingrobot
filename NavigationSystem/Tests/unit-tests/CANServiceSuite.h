////////////////////////////////////////////////////////
//
//    CAN-tests will only run when connected to hardware
//
////////////////////////////////////////////////////////
#pragma once

#include "SystemServices/Logger.h"
#include "TestMocks/MockCANReceiver.h"
#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "Hardwares/CAN_Services/CanBusCommon/CanMessageHandler.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <thread>
#include <chrono>
#include <iostream>
#include <CanMessageHandler.h>

#define WAIT_FOR_MSG 1000

class CANServiceSuite : public CxxTest::TestSuite
{
public:
  CANServiceSuite() {
    wiringPiSetup();
  }
  void setUp() {
    service = new CANService();
  }
  void tearDown()
  {
    delete service;
  }

  void test_SendAndMissedMessages()
  {
      std::vector<uint32_t> a = {700};
      MockCANReceiver receiver (*service, a );
      auto fut = service->start();
      service->SetLoopBackMode();

      CanMessageHandler messageHandler(MSG_ID_AU_CONTROL);
      for(int i=0; i<6; i++)
      {
        messageHandler.encodeMessage(1,i);
      }

      CanMsg Cmsg = messageHandler.getMessage();

      service->sendCANMessage(&Cmsg);
      bool missed = service->checkMissedMessages();

      std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MSG));
      service->SetNormalMode();

      CanMsg Cmsg2;
      N2kMsg Nmsg;

      Nmsg.PGN = 59904;
      Nmsg.Priority = 6;
      Nmsg.Source = 1;
      Nmsg.Destination = 255;
      Nmsg.DataLen = 3;
      Nmsg.Data.resize(Nmsg.DataLen);
      Nmsg.Data[0] = 20;
      Nmsg.Data[1] = 240;
      Nmsg.Data[2] = 1;

      N2kMsgToCanMsg(Nmsg, Cmsg2);

      service->sendCANMessage(Cmsg2);

      std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MSG));

      bool missed2 = service->checkMissedMessages();

      service->stop();
      fut.get();

      TS_ASSERT(receiver.message_received());
      TS_ASSERT(!missed);
      TS_ASSERT(missed2);

      if (Logger::init())
      {
		    Logger::info("Built on %s at %s", __DATE__, __TIME__);
		    Logger::info("Logger init\t\t[OK]");
	    }
	    else
      {
		    Logger::error("Logger init\t\t[FAILED]");
	    }

      if(missed2)
      {
        Logger::error("Missed messages from the CAN-Bus");
      }
      else
      {
        Logger::info("No missed messages from the CAN-Bus");
      }

      Logger::shutdown();
  }

private:
  CANService* service;
};

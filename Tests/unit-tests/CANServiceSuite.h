////////////////////////////////////////////////////////
//
//    CAN-tests will only run when connected to hardware
//
////////////////////////////////////////////////////////
#pragma once

#include "TestMocks/MockCANReceiver.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/CANFrameReceiver.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <thread>
#include <chrono>
#include <iostream>

#define WAIT_FOR_MSG 1000

class CANServiceSuite : public CxxTest::TestSuite
{
public:
  void setUp()
  {
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

      CanMsg Cmsg;
      Cmsg.id = 700;
      Cmsg.header.ide = 0;
      for(int i=0; i<8; i++)
      {
        Cmsg.data[i] = i;
      }
      Cmsg.header.length = sizeof(Cmsg.data) / sizeof(Cmsg.data[0]);

      service->sendCANMessage(Cmsg);
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

  }

private:
  CANService* service;
};

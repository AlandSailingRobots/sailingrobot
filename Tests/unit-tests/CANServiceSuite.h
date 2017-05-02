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

class CANServiceSuite : public CxxTest::TestSuite {
public:
  void setUp() {
    service = new CANService();
  }
  void tearDown() {
    delete service;
  }

  void test_CANServiceNodeCommunication () {
      MockCANReceiver receiver (*service, std::vector<uint32_t>{ 700      } );
      MockCANReceiver receiver2(*service, std::vector<uint32_t>{ 700, 701 } );
      auto fut = service->start();

      CanMsg msg;
      msg.id = 700;
      for(int i=0; i<8; i++)
      {
        msg.data[i] = i;
      }
      service->sendCANMessage(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MSG));
      service->stop();
      fut.get();

      TS_ASSERT( receiver.message_received());
      TS_ASSERT(receiver2.message_received());

  }

private:
  CANService* service;
};

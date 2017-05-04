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
    std::cout << &service << std::endl;
  }
  void tearDown() {
    std::cout << "this should not run" << std::endl;
    delete service;
  }

  void test_CANServiceNodeCommunication () {
      std::cout << "checking where the segfault is:" << std::endl;
      std::vector<uint32_t> a = {700};
      MockCANReceiver receiver (*service, a );
      std::cout << "1";
      MockCANReceiver receiver2(*service, std::vector<uint32_t>{ 700, 701 } );
      std::cout << "2";
      auto fut = service->start();
      std::cout << "3";

      CanMsg msg;
      msg.id = 700;
      for(int i=0; i<8; i++)
      {
        msg.data[i] = i;
      }
      std::cout << "4";
      service->sendCANMessage(msg);
      std::cout << "5";

      std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MSG));
      std::cout << "6";
      service->stop();
      std::cout << "7";
      fut.get();
      std::cout << "8";

      TS_ASSERT( receiver.message_received());
      std::cout << "9";
      TS_ASSERT(receiver2.message_received());
      std::cout << "10" << std::endl;

  }

private:
  CANService* service;
};

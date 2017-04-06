#include "TestMocks/MockCANReceiver.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/CANPGNReceiver.h"
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

  void test_CANServiceSendMessage() {

    MockCANReceiver receiver(*service, std::vector<uint32_t>{1304} );
    auto fut = service->start();

    N2kMsg msg;
    msg.PGN = 1304;
    std::vector<uint8_t> data = {12,24,36};
    msg.Data = data;
    service->sendN2kMessage(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MSG));
    service->stop();
    fut.get();

    TS_ASSERT(receiver.message_received());
  }

  void test_CANServiceNodeCommunication () {
      MockCANReceiver rec (*service, std::vector<uint32_t>{1337, 1339} );
      MockCANReceiver rec2(*service, std::vector<uint32_t>{1304} );
      auto fut = service->start();

      N2kMsg msg;
      msg.PGN = 1304;
      std::vector<uint8_t> data = {12,24,36};
      msg.Data = data;
      service->sendN2kMessage(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MSG));
      service->stop();
      fut.get();

      TS_ASSERT (rec.message_received());
      TS_ASSERT(rec2.message_received());

  }

private:
  CANService* service;
};

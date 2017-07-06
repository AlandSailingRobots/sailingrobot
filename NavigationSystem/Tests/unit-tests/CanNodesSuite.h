/****************************************************************************************
*
* File:
* 		CanNodesSuite.h
*
* Purpose:
*		A set of unit tests for checking if the CANAIS, CanSolarTracker and
*   CANWindsensor nodes works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "Hardwares/CANAISNode.h"
#include "Hardwares/CANSolarTrackerNode.h"
#include "Hardwares/CANWindsensorNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define CAN_TEST_COUNT 8

class CanNodesSuite : public CxxTest::TestSuite {
public:

  CANAISNode* aisNode;
  CANSolarTrackerNode* solarNode;
  CANWindsensorNode windsensorNode;

  MockNode* aisMockNode;
  MockNode* solarMockNode;
  MockNode* windMockNode;

  MockCANReceiver mockCan;

  bool aisNodeRegistered = false;
  bool solarNodeRegistererd = false;
  bool windNodeRegistered = false;

  CANService canService;

  int testCount;

  std::thread thr;

  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  static void runMessageLoop() {
    msgBus().run();
  }

  void setUp() {

    canService->start();

    aisMockNode = new MockNode(msgBus(),aisNodeRegistered);
    solarMockNode = new MockNode(msgBus(),solarNodeRegistererd);
    windMockNode = new MockNode(msgBus(),windNodeRegistered);

    mockCan = new MockCANReceiver(canService, {700, 701});

    if (aisNode == 0) {
      Logger::DisableLogging();

      aisNode = new CANAISNode(msgBus(), canService, 1.0);
      solarNode = new CANSolarTrackerNode();
      windsensorNode = new CANWindsensorNode();

      aisNode->start();
      solarNode->start();
      windsensorNode->start();

      std::this_thread::sleep_for(std::chrono::milliseconds 3000);

      thr = new std::thread(runMessageLoop());
    }
    testCount++;
  }

  void tearDown() {
    if (testCount == CAN_TEST_COUNT) {
      delete aisNode;
      delete solarNode;
      delete windsensorNode;
    }
    delete aisMockNode;
    delete solarMockNode;
    delete windMockNode;
  }

  void test_CanInit() {
    TS_ASSERT(aisNodeRegistered);
    TS_ASSERT(solarNodeRegistererd);
    TS_ASSERT(windNodeRegistered);
  }

  void test
}

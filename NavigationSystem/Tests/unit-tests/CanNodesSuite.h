/****************************************************************************************
*
* File:
* 		CanNodesSuite.h
*
* Purpose:
*   Tests the messagetypes, SolarDataMsg and AISDataMsg, works as intended
*   Sending and receiving the messages, serialising and deserialising the messages
* This test doesn't really test much yet.
*
***************************************************************************************/
#pragma once

#include "Hardwares/CANAISNode.h"
#include "Hardwares/CANSolarTrackerNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "TestMocks/MockCANReceiver.h"
#include "MessageBusTestHelper.h"

#include <chrono>

#define CAN_TEST_COUNT 2

class CanNodesSuite : public CxxTest::TestSuite {
public:

  CANAISNode* aisNode;
  DBHandler* dbhandler;
  CANSolarTrackerNode* solarNode = 0;
  MockNode* mockNode;

  bool mockNodeRegistered = false;

  CANService* canService;

  MessageBus messageBus;
  std::unique_ptr<MessageBusTestHelper> messageBusHelper;

  int testCount;

  void setUp() {
    canService = new CANService();

    if (solarNode == 0) {
      mockNode = new MockNode(messageBus, mockNodeRegistered);
      Logger::DisableLogging();

      dbhandler = new DBHandler("../asr.db");
      aisNode = new CANAISNode(messageBus,*dbhandler, *canService);
      solarNode = new CANSolarTrackerNode(messageBus, *dbhandler, *canService, 100);

      // aisNode->start();
      // solarNode->start();
      // windsensorNode->start();

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      messageBusHelper.reset(new MessageBusTestHelper(messageBus));
    }
    testCount++;
  }

  void tearDown() {
    if (testCount == CAN_TEST_COUNT) {
      delete aisNode;
      delete solarNode;
      messageBusHelper.reset();
      delete mockNode;
    }
    delete canService;
  }

  void test_CanInit() {
    TS_ASSERT(mockNodeRegistered);
  }

  void test_AISData() {
    std::vector<AISVessel> AISList;
    std::vector<AISVesselInfo> AISInfo;
    AISVessel v1, v2, v3;
    v1.MMSI = 1;
    v1.latitude = 60.2f;
    v1.longitude = 19.1f;
    v1.COG = 200;
    v1.SOG = 10;
    v2.MMSI = 2;
    v2.latitude = 62.f;
    v2.longitude = 18.1f;
    v2.COG = 100;
    v2.SOG = 5;
    v3.MMSI = 3;
    v3.latitude = 61.5f;
    v3.longitude = 18.7f;
    v3.COG = 80;
    v3.SOG = 7;
    AISList.push_back(v1);
    AISList.push_back(v2);
    AISList.push_back(v3);
    AISVesselInfo i1;
    i1.MMSI=1;
    i1.length=15;
    i1.beam = 4;
    AISInfo.push_back(i1);


    MessagePtr mockAISMsg = std::make_unique<AISDataMsg>(AISList, AISInfo, 60.1, 19.1);
    messageBus.sendMessage(std::move(mockAISMsg));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    TS_ASSERT(mockNode->m_MessageReceived);
  }
};

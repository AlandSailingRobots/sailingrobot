/****************************************************************************************
*
* File:
* 		CanNodesSuite.h
*
* Purpose:
*   Tests the messagetypes, SolarDataMsg and AISDataMsg, works as intended
*   Sending and receiving the messages, serialising and deserialising the messages
*
***************************************************************************************/
#pragma once

#include "Hardwares/CANAISNode.h"
#include "Hardwares/CANSolarTrackerNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "TestMocks/MockCANReceiver.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define CAN_TEST_COUNT 4

class CanNodesSuite : public CxxTest::TestSuite {
public:

  CANAISNode* aisNode;
  CANSolarTrackerNode* solarNode;
  MockNode* mockNode;

  bool mockNodeRegistered = false;

  CANService* canService;

  int testCount;

  std::thread* thr;

  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  static void runMessageLoop() {
    msgBus().run();
  }

  void setUp() {
    canService = new CANService();
    mockNode = new MockNode(msgBus(), mockNodeRegistered);

    if (solarNode == 0) {
      Logger::DisableLogging();

      aisNode = new CANAISNode(msgBus(), *canService, 0.51);
      // solarNode = new CANSolarTrackerNode(msgBus(), *canService, 100);

      // aisNode->start();
      // solarNode->start();
      // windsensorNode->start();

      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      thr = new std::thread(runMessageLoop);
    }
    testCount++;
  }

  void tearDown() {
    if (testCount == CAN_TEST_COUNT) {
      thr->detach();
      delete thr;
      delete aisNode;
      delete solarNode;
    }
    delete mockNode;
    delete canService;
  }

  void test_CanInit() {
    TS_ASSERT(mockNodeRegistered);
  }

  void test_MessageListening() {
    double heading = 200;
    double latitude = 60.2;
    double longitude = 19.1;
    int hour = 12;
    int min = 15;

    MessagePtr mockSolarMsg = std::make_unique<SolarDataMsg>(latitude,longitude,heading,hour,min);
    msgBus().sendMessage(std::move(mockSolarMsg));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    TS_ASSERT(mockNode->m_MessageReceived);
  }

  void test_SolarData() {
    double heading = 200;
    double latitude = 60.2;
    double longitude = 19.1;

    MessagePtr mockSolarMsg = std::make_unique<StateMessage>(heading,latitude,longitude,1,20);
    msgBus().sendMessage(std::move(mockSolarMsg));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    TS_ASSERT_DELTA(mockNode->m_latitude, latitude, 1e-3);
    TS_ASSERT_DELTA(mockNode->m_longitude, longitude, 1e-3);
    TS_ASSERT_DELTA(mockNode->m_heading, heading, 1e-3);
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
    msgBus().sendMessage(std::move(mockAISMsg));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    TS_ASSERT(mockNode->m_MessageReceived);
  }
};

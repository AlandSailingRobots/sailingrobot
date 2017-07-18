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
// #include "Hardwares/CANWindsensorNode.h"
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
  // CANWindsensorNode* windsensorNode;

  MockNode* mockNode;
  // MockNode* windMockNode;

  // MockCANReceiver mockCan;

  bool mockNodeRegistered = false;
  // bool windNodeRegistered = false;

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
    // windMockNode = new MockNode(msgBus(),windNodeRegistered);

    // std::vector<uint32_t> canMessages; // = {700, 701};
    // canMessages[0] = (uint32_t) 700;
    // canMessages[1] = (uint32_t) 701;
    // mockCan = new MockCANReceiver(*canService, *canMessages);
    // canService->();
    if (solarNode == 0) {
      Logger::DisableLogging();

      aisNode = new CANAISNode(msgBus(), *canService, 500);
      //solarNode = new CANSolarTrackerNode(msgBus(), *canService, 100);
      // windsensorNode = new CANWindsensorNode(msgBus(), *canService, 1.0);

      aisNode->start();
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
    // TS_ASSERT(windNodeRegistered);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    TS_ASSERT(mockNode->m_MessageReceived);
  }

};

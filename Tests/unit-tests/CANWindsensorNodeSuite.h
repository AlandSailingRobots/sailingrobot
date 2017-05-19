/****************************************************************************************
*
* File:
* 		CANWindsensorNodeSuite.h
*
* Purpose:
*		A set of unit tests seeing if the CANWindowsensorNodeSuite works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "Nodes/CANWindsensorNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "Messages/VesselStateMsg.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define CANWINDSENSORNODE_TEST_COUNT   4


class CANWindsensorNodeSuite : public CxxTest::TestSuite
{
public:

  CANWindsensorNode* can_windSensorNode;
  std::thread* thr;
  MessageLogger* logger;
  int testCount = 0;
  std::vector<uint8_t> someTestdata;

  // Cheeky method for declaring and initialising a static in a header file
  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  static CANService& canService(){
    static CANService* cservice = new CANService();
    return *cservice;
  }

  static std::vector<uint8_t>& getUint8_tTestData(){
    static std::vector<uint8_t>* uint8_tData = new std::vector<uint8_t>();
    uint8_t data;
    for(int i = 0; i < 5; i++){
      data = rand() % 256;
      uint8_tData->push_back(data);
    }
    return* uint8_tData;
  }

  static void runMessageLoop()
  {
    msgBus().run();
  }

  void setUp()
  {
    // Only want to setup them up once in this test, only going to delete them when the program closes and the OS destroys
    // the process's memory
    if(can_windSensorNode == 0)
    {
      Logger::DisableLogging();
      logger = new MessageLogger(msgBus());
      can_windSensorNode = new CANWindsensorNode(msgBus(), canService(), 50);
      can_windSensorNode->start();
      srand (time(NULL));
      someTestdata = getUint8_tTestData();

      thr = new std::thread(runMessageLoop);

    }
    testCount++;
    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
  }

  void tearDown()
  {
    if(testCount == CANWINDSENSORNODE_TEST_COUNT)
    {
      delete can_windSensorNode;
      delete logger;
    }
  }

  void test_CANWindsensorNodeInit()
  {
    TS_ASSERT(can_windSensorNode->init())
  }

  void test_CANWindsensorNodeprocessPGN130306()
  {
    N2kMsg msg = {.PGN = 130306, .Priority = 1,
    .Source = 10, .Destination = 2,
    .DataLen = 5, .Data = someTestdata};
    can_windSensorNode->processPGN(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
    TS_ASSERT(logger->windDataReceived());
  }

  void test_CANWindsensorNodeprocessPGN130311()
  {
    N2kMsg msg = {.PGN = 130311, .Priority = 1,
    .Source = 10, .Destination = 2,
    .DataLen = 5, .Data = someTestdata};
    can_windSensorNode->processPGN(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
    TS_ASSERT(logger->windDataReceived());
  }

  void test_CANWindsensorNodeparsePGN130306()
  {
    N2kMsg msg = {.PGN = 130306, .Priority = 1,
    .Source = 10, .Destination = 2,
    .DataLen = 5, .Data = someTestdata};

    for (int i = 0; i < 1000; i++){
      uint8_t SID, Ref;
      float WS, WA;

      can_windSensorNode->parsePGN130306(msg, SID, WS, WA, Ref);

      TS_ASSERT(WA < 360 && WA > 0);
      TS_ASSERT(WS > 0);
      msg.Data = getUint8_tTestData();
    }
  }
};

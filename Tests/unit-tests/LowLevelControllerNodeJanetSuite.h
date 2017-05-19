/****************************************************************************************
*
* File:
* 		LineFollowSuite.h
*
* Purpose:
*		A set of unit tests seeing if the LineFollowNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "../cxxtest/cxxtest/TestSuite.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Nodes/LowLevelControllerNodeJanet.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define TEST_COUNT   2

class LowLevelControllerNodeJanetSuite : public CxxTest::TestSuite
{
public:
  LowLevelControllerNodeJanet* janet;

  std::thread* thr;
  MessageLogger* logger;
  int testCount = 0;

  // Cheeky method for declaring and initialising a static in a header file
  static MessageBus& msgBus(){
    static MessageBus* mbus = new MessageBus();
    return *mbus;
  }

  static void runMessageLoop()
  {
    msgBus().run();
  }

  void setUp()
  {
    // setup them up once in this test, delete them when the program closes
    if(janet == 0)
    {
      Logger::DisableLogging();
      logger = new MessageLogger(msgBus());
      srand (time(NULL));

      janet = new LowLevelControllerNodeJanet(msgBus());
      thr = new std::thread(runMessageLoop);
    }
    testCount++;
    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
  }

  void tearDown()
  {
    if(testCount == TEST_COUNT)
    {
      delete janet;
      delete logger;
    }
  }

  void test_LowLevelControllerNodeJanetInit(){
    TS_ASSERT(janet->init());
  }

  void test_LowLevelCntNodeJanetActuatorPosMsgSent(){
    msgBus().sendMessage(std::make_unique<NavigationControlMsg>(1.2, 2.3, false, NavigationState::sailToWaypoint));
    msgBus().sendMessage(std::make_unique<StateMessage>(1,2,3,4,5));
    msgBus().sendMessage(std::make_unique<WindStateMsg>(6,7,8,9));
    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
    TS_ASSERT(logger->actuatorPositionReceived());
  }

};

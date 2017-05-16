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
      thr = new std::thread(runMessageLoop);
    }
    janet = new LowLevelControllerNodeJanet(msgBus());
    testCount++;
  }

  void tearDown()
  {
    delete janet;
    if(testCount == TEST_COUNT)
    {
      delete logger;
    }
  }

  void test_LowLevelControllerNodeJanetInit(){
    TS_ASSERT(janet->init());
  }

  void test_LowLevelControllerNodeJanetActuatorPosMsgSent(){
    msgBus().sendMessage(std::make_unique<NavigationControlMsg>(1.2, 2.3, false, NavigationState::sailToWaypoint));
    msgBus().sendMessage(std::make_unique<StateMessage>(1,2,3,4,5));
    msgBus().sendMessage(std::make_unique<WindStateMsg>(6,7,8,9));

    TS_ASSERT(janet->init());

  }

};

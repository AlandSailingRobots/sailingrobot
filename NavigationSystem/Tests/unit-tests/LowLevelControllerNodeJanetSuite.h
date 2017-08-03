/****************************************************************************************
*
* File:
* 		StateEstimationNodeSuite.h
*
* Purpose:
*		A set of unit tests for checking if the StateEstimationNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "LowLevelControllers/LowLevelControllerNodeJanet.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "TestMocks/MockNode.h"


// For std::this_thread
#include <chrono>
#include <thread>

#define NODE_JANET_TEST_COUNT   3


class LowLevelControllerNodeJanetSuite : public CxxTest::TestSuite
{
public:

  LowLevelControllerNodeJanet* nodeJanet;

  MockNode* mockNode;
  bool nodeRegistered = false;
  std::thread* thr;
  int testCount = 0;
  DBHandler* dbHandler;


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
    if(nodeJanet == 0)
    {
      dbHandler = new DBHandler("./asr.db");
      Logger::DisableLogging();
      mockNode = new MockNode(msgBus(), nodeRegistered);

      nodeJanet = new LowLevelControllerNodeJanet(msgBus(), 30, 60, *dbHandler);

      thr = new std::thread(runMessageLoop);
    }
    testCount++;
  }

  void tearDown()
  {
    if(testCount == NODE_JANET_TEST_COUNT)
    {
        msgBus().stop();
        thr -> join();
        delete thr;
        delete nodeJanet;
        delete dbHandler;
        delete mockNode;
    }
  }

  void test_LowLevelControllerNodeJanetRegistered(){
    TS_ASSERT(nodeRegistered);
  }

  void test_LowLevelControllerNodeJanetInit(){
    TS_ASSERT(nodeJanet->init());
  }

  void test_LowLeveControllerNodeProcessMsg(){

    NavigationState state = NavigationState::sailToWaypoint;

    MessagePtr msg = std::make_unique<WindStateMsg>(170, 30, 0, 200);
    MessagePtr sMsg = std::make_unique<StateMessage>(170, 30, 10, 200, 50);
    MessagePtr nMsg = std::make_unique<NavigationControlMsg>(1.2, 2.3, false, false, false, state);

    msgBus().sendMessage(std::move(msg));
    msgBus().sendMessage(std::move(sMsg));
    msgBus().sendMessage(std::move(nMsg));

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    TS_ASSERT(mockNode->m_MessageReceived);
  }

};

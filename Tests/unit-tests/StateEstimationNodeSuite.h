/****************************************************************************************
*
* File:
* 		StateEstimationNodeSuite.h
*
* Purpose:
*		A set of unit tests seeing if the StateEstimationNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "Nodes/StateEstimationNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"


// For std::this_thread
#include <chrono>
#include <thread>

#define STATE_ESTIMATIONODE_TEST_COUNT   3
#define SLEEP_TIME 2005


class StateEstimationNodeSuite : public CxxTest::TestSuite
{
public:

  StateEstimationNode* sEstimationNode;
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
    // Only want to setup them up once in this test, only going to delete them when the program closes and the OS destroys
    // the process's memory
    if(sEstimationNode == 0)
    {
      Logger::DisableLogging();
      logger = new MessageLogger(msgBus());
      sEstimationNode = new StateEstimationNode(msgBus(), 1, 1);
      srand (time(NULL));
      thr = new std::thread(runMessageLoop);
    }
    testCount++;
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  }

  void tearDown()
  {
    if(testCount == STATE_ESTIMATIONODE_TEST_COUNT)
    {
      delete sEstimationNode;
      delete logger;
    }
  }

  void test_StateEstimationNodeInit()
  {
    TS_ASSERT(sEstimationNode->init());
  }

  void test_StateEstimationNodeStateMsgReceived(){
    sEstimationNode->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
    TS_ASSERT(logger->StateDataReceived());
  }

};

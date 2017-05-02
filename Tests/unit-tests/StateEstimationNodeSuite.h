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

#include "Nodes/StateEstimationNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "TestMocks/StateMessageListener.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"


// For std::this_thread
#include <chrono>
#include <thread>

#define STATE_ESTIMATIONODE_TEST_COUNT   3


class StateEstimationNodeSuite : public CxxTest::TestSuite
{
public:

  StateEstimationNode* sEstimationNode;
  StateMessageListener* stateMessageListener;

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
    if(sEstimationNode == 0)
    {
      Logger::DisableLogging();
      /*gpsMsgData = std::make_unique<GPSDataMsg>(logger->nodeID(),
          sEstimationNode->nodeID(), false, false,60.09726,19.93481,1,1,10,2,
          GPSMode::LatLonOk);*/
      logger = new MessageLogger(msgBus());
      stateMessageListener = new StateMessageListener(msgBus());
      sEstimationNode = new StateEstimationNode(msgBus(), .5, 1);
      srand (time(NULL));
      thr = new std::thread(runMessageLoop);
    }
    testCount++;
  }

  void tearDown()
  {
    if(testCount == STATE_ESTIMATIONODE_TEST_COUNT)
    {
      delete sEstimationNode;
      delete logger;
      delete stateMessageListener;
    }
  }

  void test_StateEstimationNodeInit()
  {
    TS_ASSERT(sEstimationNode->init());
  }

  void test_StateEstimationNodeStateMsgReceived(){
    sEstimationNode->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2600));
    TS_ASSERT(logger->stateDataReceived());
    logger->clearState();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    TS_ASSERT(logger->stateDataReceived());
    TS_ASSERT(logger->gpsDataReceived());
  }

  void test_stateMessageListener(){
    TS_ASSERT(stateMessageListener->init());
    TS_ASSERT(stateMessageListener->stateDataReceived());
    TS_ASSERT(stateMessageListener->getVesselSpeed() == 0);
  }

};

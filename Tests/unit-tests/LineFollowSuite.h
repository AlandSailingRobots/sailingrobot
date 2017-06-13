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
*							12.4.17 JM
*
*	Functions that have tests:		Functions that does not have tests:
*
*	init 						calculateAngleOfDesiredTrajectory
*									calculateActuatorPos
*									setPrevWaypointData
*									getHeading
*									getMergedHeading
*									setupRudderCommand
*									setupSailCommand
*									getGoingStarboard
*									setPrevWaypointToBoatPos
*
***************************************************************************************/

#pragma once

#include "../cxxtest/cxxtest/TestSuite.h"
#include "../../MessageBus/MessageBus.h"
#include "Nodes/LineFollowNode.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/SoftsailControl.h"
#include "Math/Utility.h"


// For std::this_thread
#include <chrono>
#include <thread>
#include <math.h>

#define WAIT_FOR_MESSAGE		300
#define LINEFOLLOW_TEST_COUNT   2

class LineFollowSuite : public CxxTest::TestSuite {
public:
  LineFollowNode* lineFollow;
  std::thread* thr;

  MockNode* mockNode;
  bool nodeRegistered = false;

  int testCount = 0;
  DBHandler* dbHandler;

  // Cheeky method for declaring and initialising a static in a header file
  static MessageBus& msgBus()
  {
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
    if(lineFollow == 0)
    {
      dbHandler = new DBHandler("./asr.db");
      Logger::DisableLogging();
      lineFollow = new LineFollowNode(msgBus(), *dbHandler);

      thr = new std::thread(runMessageLoop);
    }
    mockNode = new MockNode(msgBus(), nodeRegistered);
    testCount++;
    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
  }

  void tearDown()
  {
    if(testCount == LINEFOLLOW_TEST_COUNT)
    {
      delete lineFollow;
      delete dbHandler;
    }
    delete mockNode;

  }

  void test_LineFollowInit()
  {
    TS_ASSERT(lineFollow->init());
    TS_ASSERT(nodeRegistered);
  }

  void test_LineFollowCalculateActuatorPosition()
  {

    MessagePtr msg = std::make_unique<WindStateMsg>(170, 30, 0, 200);
    MessagePtr sMsg = std::make_unique<StateMessage>(170, 30, 10, 200, 50);

    msgBus().sendMessage(std::move(msg));
    msgBus().sendMessage(std::move(sMsg));

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    TS_TRACE("END OF LINEFOLLOW");
    TS_ASSERT(mockNode->m_MessageReceived);
  }


  };

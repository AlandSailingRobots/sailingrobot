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

#include "../../Math/Utility.h"
#include "../../MessageBus/MessageBus.h"
#include "MessageBusTestHelper.h"
#include "../../Messages/StateMessage.h"
#include "../../Messages/VesselStateMsg.h"
#include "../../Messages/WindStateMsg.h"
#include "../../Navigation/LineFollowNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"

#include <math.h>
#include <chrono>

#define LINEFOLLOW_TEST_COUNT 2

class LineFollowSuite : public CxxTest::TestSuite {
   public:
    LineFollowNode* lineFollow;

    MockNode* mockNode;
    bool nodeRegistered = false;

    int testCount = 0;
    DBHandler* dbHandler;
    const int WAIT_FOR_MESSAGE = 500;
    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    void setUp() {
        // Only want to setup them up once in this test, only going to delete them when the program
        // closes and the OS destroys the process's memory
        if (lineFollow == 0) {
            mockNode = new MockNode(messageBus, nodeRegistered);
            dbHandler = new DBHandler("../asr.db");
            Logger::DisableLogging();
            lineFollow = new LineFollowNode(messageBus, *dbHandler);
            lineFollow->start();
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        }
        testCount++;
    }

    void tearDown() {
        if (testCount == LINEFOLLOW_TEST_COUNT) {
            lineFollow->stop();
            messageBusHelper.reset();
            delete lineFollow;
            delete dbHandler;
            delete mockNode;
        }
    }

    void test_LineFollowInit() {
        TS_ASSERT(lineFollow->init());
        TS_ASSERT(nodeRegistered);
    }

    void test_LineFollowCalculateActuatorPosition() {
        // WindState no register in the mocknode
        MessagePtr msg = std::make_unique<WindStateMsg>(170, 30, 0, 200);
        MessagePtr sMsg = std::make_unique<StateMessage>(170, 30, 10, 200, 50);
        messageBus.sendMessage(std::move(msg));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        messageBus.sendMessage(std::move(sMsg));
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
        TS_TRACE("END OF LINEFOLLOW");
        TS_ASSERT(mockNode->m_MessageReceived);
    }
};

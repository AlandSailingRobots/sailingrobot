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
#include "Nodes/LineFollowNode.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"

// For std::this_thread
#include <chrono>
#include <thread>

#define WAIT_FOR_MESSAGE		300
#define LINEFOLLOW_TEST_COUNT   2

class LineFollowSuite : public CxxTest::TestSuite {
    public:
    LineFollowNode* lineFollow;
    std::thread* thr;
	MessageLogger* logger;
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
			logger = new MessageLogger(msgBus());
			lineFollow = new LineFollowNode(msgBus(), *dbHandler);
			thr = new std::thread(runMessageLoop);
		}
		testCount++;
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));
	}

	void tearDown()
	{
		if(testCount == LINEFOLLOW_TEST_COUNT)
		{
			delete lineFollow;
			delete logger;
			delete dbHandler;
		}
	}

    void test_LineFollowInit()
    {
        TS_ASSERT(lineFollow->init());
    }

    void test_LineFollowCalculateActuatorPosition()
    {

        MessagePtr msg = std::make_unique<WindStateMsg>(170, 30, 0, 200);
        MessagePtr sMsg = std::make_unique<StateMessage>(170, 30, 10, 200, 50);

        msgBus().sendMessage(std::move(msg));
        msgBus().sendMessage(std::move(sMsg));

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		TS_TRACE("END OF LINEFOLLOW");
        TS_ASSERT(logger->navigationDataReceived());
    }

};
